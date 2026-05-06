#!/usr/bin/env python3
"""
6-DOF rigid-body rocket dynamics with quaternion attitude representation.
Solves Newton-Euler equations: F=m*dv/dt, I*dω/dt + ω×(I×ω) = M, dq/dt = 0.5*Ω*q.
"""

import numpy as np
import warnings

from dynamics.quaternion_utils import (
    quaternion_to_rotation_matrix as _quaternion_to_rotation_matrix,
    DegenerateQuaternionError,
)


class EarthModel:
    """WGS-84 Earth model with gravity options: constant, somigliana, or J2 perturbation."""
    
    WGS84_A = 6378137.0          # WGS84 equatorial radius (m)
    WGS84_B = 6356752.314245     # WGS84 polar radius (m)
    WGS84_F = 1.0 / 298.257223563  # WGS84 flattening
    WGS84_E2 = 0.00669437999014  # WGS84 first eccentricity squared

    MU = 3.986004418e14  # Earth gravitational constant (m³/s²)
    J2 = 1.08263e-3      # Earth oblateness coefficient
    J3 = -2.53215e-6     # Earth oblateness coefficient
    J4 = -1.6204e-6      # Vallado "Fundamentals of Astrodynamics and Applications"
    OMEGA_E = 7.2921159e-5  # Earth rotation rate (rad/s)

    G0 = 9.80665  # Standard gravity (m/s²)

    # Alias for compatibility with frame_manager.py
    equatorial_radius = WGS84_A
    
    def __init__(self, origin_lla=None):
        """
        Initialize Earth model.
        
        Args:
            origin_lla: Launch site [lat_rad, lon_rad, alt_m] or None for [0, 0, 0]
        """
        if origin_lla is None:
            self.origin_lla = np.array([0.0, 0.0, 0.0])
        else:
            self.origin_lla = np.array(origin_lla)
        
        self.origin_ecef = self.lla_to_ecef(
            self.origin_lla[0], self.origin_lla[1], self.origin_lla[2]
        )
        self.C_ecef_to_ned = self.rot_ecef_to_ned(
            self.origin_lla[0], self.origin_lla[1]
        )
    
    def lla_to_ecef(self, lat, lon, alt):
        """
        Convert geodetic coordinates (LLA) to ECEF.
        
        Args:
            lat: Geodetic latitude (radians)
            lon: Geodetic longitude (radians)
            alt: Altitude above ellipsoid (meters)
            
        Returns:
            np.array: ECEF position [x, y, z] in meters
        """
        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        sin_lon = np.sin(lon)
        cos_lon = np.cos(lon)
        
        N = self.WGS84_A / np.sqrt(1.0 - self.WGS84_E2 * sin_lat**2)
        
        x = (N + alt) * cos_lat * cos_lon
        y = (N + alt) * cos_lat * sin_lon
        z = (N * (1.0 - self.WGS84_E2) + alt) * sin_lat
        
        return np.array([x, y, z])
    
    def ecef_to_lla(self, x, y, z, tol=1e-12, max_iter=10):
        """Convert ECEF (x,y,z) to geodetic (lat_rad, lon_rad, alt_m) iteratively."""
        lon = np.arctan2(y, x)
        
        p = np.sqrt(x**2 + y**2)
        lat = np.arctan2(z, p * (1.0 - self.WGS84_E2))
        
        for _ in range(max_iter):
            sin_lat = np.sin(lat)
            N = self.WGS84_A / np.sqrt(1.0 - self.WGS84_E2 * sin_lat**2)
            lat_new = np.arctan2(z + self.WGS84_E2 * N * sin_lat, p)
            
            if abs(lat_new - lat) < tol:
                lat = lat_new
                break
            lat = lat_new
        
        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        N = self.WGS84_A / np.sqrt(1.0 - self.WGS84_E2 * sin_lat**2)
        
        if abs(cos_lat) > 1e-10:
            alt = p / cos_lat - N
        else:
            alt = abs(z) / abs(sin_lat) - N * (1.0 - self.WGS84_E2)
        
        return lat, lon, alt
    
    def rot_ecef_to_ned(self, lat, lon):
        """3x3 rotation matrix from ECEF to NED frame for given lat/lon (radians)."""
        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        sin_lon = np.sin(lon)
        cos_lon = np.cos(lon)
        
        C = np.array([
            [-sin_lat * cos_lon, -sin_lat * sin_lon,  cos_lat],
            [-sin_lon,            cos_lon,            0.0],
            [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat]
        ])
        
        return C
    
    def ecef_to_ned_rotation(self, lat, lon):
        """Alias for rot_ecef_to_ned."""
        return self.rot_ecef_to_ned(lat, lon)
    
    def ned_to_ecef(self, pos_ned):
        """Convert NED position (relative to origin) to ECEF."""
        pos_ecef = self.origin_ecef + self.C_ecef_to_ned.T @ pos_ned
        return pos_ecef
    
    def gravity_constant(self, mass):
        """Constant gravity force in NED frame (Newtons)."""
        return np.array([0.0, 0.0, self.G0 * mass])
    
    def gravity_somigliana(self, lat, alt, mass):
        """Somigliana gravity force in NED frame (latitude/altitude dependent)."""
        sin_lat2 = np.sin(lat)**2
        
        g_equator = 9.7803253359  # Gravity at equator (m/s²)
        k = 0.00193185265241      # Gravity formula constant
        e2 = 0.00669437999014     # First eccentricity squared
        
        g_surface = g_equator * (1.0 + k * sin_lat2) / np.sqrt(1.0 - e2 * sin_lat2)
        
        # More accurate altitude correction using inverse square law
        # (a/(a+alt))² is more accurate than (1 - 2*alt/a) at high altitudes
        g = g_surface * (self.WGS84_A / (self.WGS84_A + alt))**2
        
        return np.array([0.0, 0.0, g * mass])
    
    def gravity_j2(self, pos_ecef, mass):
        """J2 gravity force in NED frame with Earth oblateness effects."""
        x, y, z = pos_ecef
        r = np.sqrt(x**2 + y**2 + z**2)
        
        if r < 1e6:
            return self.gravity_constant(mass)
        
        r2 = r * r
        
        re_r = self.WGS84_A / r
        re_r2 = re_r * re_r
        
        # z_r = sin(phi_geocentric) - Geocentric latitude for spherical harmonics
        z_r = z / r
        z_r2 = z_r * z_r
        
        mu_r2 = self.MU / r2
        j2_term = 1.5 * self.J2 * re_r2
        
        g_ecef = np.array([
            -mu_r2 * (x / r) * (1.0 + j2_term * (1.0 - 5.0 * z_r2)),
            -mu_r2 * (y / r) * (1.0 + j2_term * (1.0 - 5.0 * z_r2)),
            -mu_r2 * (z / r) * (1.0 + j2_term * (3.0 - 5.0 * z_r2))
        ])
        
        lat, lon, _ = self.ecef_to_lla(x, y, z)
        C_ecef_to_ned_local = self.rot_ecef_to_ned(lat, lon)
        g_ned = C_ecef_to_ned_local @ g_ecef
        
        return g_ned * mass
    
    def gravity_j2_j3_j4(self, pos_ecef, mass):
        """J2-J3-J4 gravity force in NED frame for long-range trajectories."""
        x, y, z = pos_ecef
        r = np.sqrt(x**2 + y**2 + z**2)
        
        if r < 1e6:
            return self.gravity_constant(mass)
        
        r2 = r * r
        r3 = r2 * r
        r4 = r2 * r2
        
        Re = self.WGS84_A
        Re2 = Re * Re
        Re3 = Re2 * Re
        Re4 = Re2 * Re2
        
        # sin_phi = GEOCENTRIC latitude (NOT geodetic) - correct for spherical harmonics
        sin_phi = z / r
        sin_phi2 = sin_phi * sin_phi
        sin_phi3 = sin_phi2 * sin_phi
        sin_phi4 = sin_phi2 * sin_phi2
        
        mu_r2 = self.MU / r2
        
        j2_radial = 1.5 * self.J2 * (Re2 / r2) * (1.0 - 3.0 * sin_phi2)
        j3_radial = 2.0 * self.J3 * (Re3 / r3) * (5.0 * sin_phi3 - 3.0 * sin_phi)
        j4_radial = (5.0 / 8.0) * self.J4 * (Re4 / r4) * (35.0 * sin_phi4 - 30.0 * sin_phi2 + 3.0)
        
        g_r = -mu_r2 * (1.0 + j2_radial + j3_radial + j4_radial)
        
        j2_phi = 3.0 * self.J2 * (Re2 / r2) * sin_phi * np.sqrt(1.0 - sin_phi2)
        j3_phi = 1.5 * self.J3 * (Re3 / r3) * (5.0 * sin_phi2 - 1.0) * np.sqrt(1.0 - sin_phi2)
        j4_phi = (5.0 / 2.0) * self.J4 * (Re4 / r4) * sin_phi * (7.0 * sin_phi2 - 3.0) * np.sqrt(1.0 - sin_phi2)
        
        g_phi = -mu_r2 * (j2_phi + j3_phi + j4_phi)
        
        cos_phi = np.sqrt(1.0 - sin_phi2)
        if cos_phi < 1e-10:
            cos_phi = 1e-10
        
        p = np.sqrt(x**2 + y**2)
        if p < 1e-10:
            g_ecef = np.array([0.0, 0.0, g_r * np.sign(z)])
        else:
            cos_lon = x / p
            sin_lon = y / p
            
            g_x = g_r * (x / r) - g_phi * (z * cos_lon / r)
            g_y = g_r * (y / r) - g_phi * (z * sin_lon / r)
            g_z = g_r * (z / r) + g_phi * (p / r)
            
            g_ecef = np.array([g_x, g_y, g_z])
        
        lat, lon, _ = self.ecef_to_lla(x, y, z)
        C_ecef_to_ned_local = self.rot_ecef_to_ned(lat, lon)
        g_ned = C_ecef_to_ned_local @ g_ecef
        
        return g_ned * mass
    
    def gravity_ecef(self, pos_ecef, model="j2"):
        """Compute gravity acceleration in ECEF frame (m/s²). Model: "constant", "j2", or "j2_j3_j4"."""
        x, y, z = pos_ecef
        r = np.sqrt(x**2 + y**2 + z**2)
        
        if r < 1e6:
            lat, lon, _ = self.ecef_to_lla(x, y, z)
            C_ned_to_ecef = self.rot_ecef_to_ned(lat, lon).T
            return C_ned_to_ecef @ np.array([0.0, 0.0, self.G0])
        
        if model == "constant":
            lat, lon, _ = self.ecef_to_lla(x, y, z)
            C_ned_to_ecef = self.rot_ecef_to_ned(lat, lon).T
            return C_ned_to_ecef @ np.array([0.0, 0.0, self.G0])
        
        r2 = r * r
        Re = self.WGS84_A
        Re2 = Re * Re
        
        # sin_phi = GEOCENTRIC latitude - correct for spherical harmonics
        sin_phi = z / r
        sin_phi2 = sin_phi * sin_phi
        
        mu_r2 = self.MU / r2
        
        j2_radial = 1.5 * self.J2 * (Re2 / r2) * (1.0 - 3.0 * sin_phi2)
        g_r = -mu_r2 * (1.0 + j2_radial)
        
        j2_phi = 3.0 * self.J2 * (Re2 / r2) * sin_phi * np.sqrt(1.0 - sin_phi2)
        g_phi = -mu_r2 * j2_phi
        
        if model == "j2_j3_j4":
            r3 = r2 * r
            r4 = r2 * r2
            Re3 = Re2 * Re
            Re4 = Re2 * Re2
            
            sin_phi3 = sin_phi2 * sin_phi
            sin_phi4 = sin_phi2 * sin_phi2
            
            j3_radial = 2.0 * self.J3 * (Re3 / r3) * (5.0 * sin_phi3 - 3.0 * sin_phi)
            j4_radial = (5.0 / 8.0) * self.J4 * (Re4 / r4) * (35.0 * sin_phi4 - 30.0 * sin_phi2 + 3.0)
            g_r = -mu_r2 * (1.0 + j2_radial + j3_radial + j4_radial)
            
            j3_phi = 1.5 * self.J3 * (Re3 / r3) * (5.0 * sin_phi2 - 1.0) * np.sqrt(1.0 - sin_phi2)
            j4_phi = (5.0 / 2.0) * self.J4 * (Re4 / r4) * sin_phi * (7.0 * sin_phi2 - 3.0) * np.sqrt(1.0 - sin_phi2)
            g_phi = -mu_r2 * (j2_phi + j3_phi + j4_phi)
        
        p = np.sqrt(x**2 + y**2)
        if p < 1e-10:
            return np.array([0.0, 0.0, g_r * np.sign(z)])
        
        cos_lon = x / p
        sin_lon = y / p
        
        g_x = g_r * (x / r) - g_phi * (z * cos_lon / r)
        g_y = g_r * (y / r) - g_phi * (z * sin_lon / r)
        g_z = g_r * (z / r) + g_phi * (p / r)
        
        return np.array([g_x, g_y, g_z])
    
    def atmosphere_velocity_ecef(self, pos_ecef):
        """Atmosphere velocity in ECEF frame (always zero - co-rotating with Earth)."""
        return np.array([0.0, 0.0, 0.0])
    
    def coriolis_acceleration(self, velocity_ecef):
        """Coriolis acceleration in ECEF frame: a = -2 × omega × v."""
        omega = np.array([0.0, 0.0, self.OMEGA_E])
        return -2.0 * np.cross(omega, velocity_ecef)
    
    def centrifugal_acceleration(self, pos_ecef):
        """Centrifugal acceleration in ECEF frame: a = -omega × (omega × r)."""
        omega = np.array([0.0, 0.0, self.OMEGA_E])
        return -np.cross(omega, np.cross(omega, pos_ecef))
    
    def ned_frame_rotation_rate(self, lat, lon, alt, vel_ned):
        """NED frame rotation rate = Earth rotation + transport rate (rad/s)."""
        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        
        N = self.WGS84_A / np.sqrt(1.0 - self.WGS84_E2 * sin_lat**2)
        R_E = N + alt
        R_M = N * (1.0 - self.WGS84_E2) / (1.0 - self.WGS84_E2 * sin_lat**2) + alt
        
        V_N, V_E, V_D = vel_ned
        
        omega_earth_ned = np.array([
            self.OMEGA_E * cos_lat,
            0.0,
            -self.OMEGA_E * sin_lat
        ])
        
        if abs(cos_lat) < 1e-10:
            omega_transport = np.array([0.0, 0.0, 0.0])
        else:
            omega_transport = np.array([
                V_E / R_E,
                -V_N / R_M,
                -V_E * np.tan(lat) / R_E
            ])
        
        return omega_earth_ned + omega_transport
    
    def earth_rotation_rate_ned(self, lat):
        """Earth rotation rate in NED frame (without transport rate)."""
        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        
        return np.array([
            self.OMEGA_E * cos_lat,
            0.0,
            -self.OMEGA_E * sin_lat
        ])
    
    def geodesic_distance(self, lat1, lon1, lat2, lon2):
        """Geodesic distance (m) between two points using Vincenty's formula (haversine fallback)."""
        f = self.WGS84_F
        a = self.WGS84_A
        b = self.WGS84_B
        
        U1 = np.arctan((1 - f) * np.tan(lat1))
        U2 = np.arctan((1 - f) * np.tan(lat2))
        L = lon2 - lon1
        
        sin_U1 = np.sin(U1)
        cos_U1 = np.cos(U1)
        sin_U2 = np.sin(U2)
        cos_U2 = np.cos(U2)
        
        lam = L
        converged = False
        for _ in range(100):
            sin_lam = np.sin(lam)
            cos_lam = np.cos(lam)
            
            sin_sigma = np.sqrt((cos_U2 * sin_lam)**2 + 
                               (cos_U1 * sin_U2 - sin_U1 * cos_U2 * cos_lam)**2)
            
            if sin_sigma < 1e-12:
                # sin_sigma ≈ 0 can occur for two cases:
                # 1. Coincident points (same location): cos_sigma > 0, σ ≈ 0
                # 2. Antipodal points (opposite sides of Earth): cos_sigma < 0, σ ≈ π
                cos_sigma = sin_U1 * sin_U2 + cos_U1 * cos_U2 * cos_lam
                if cos_sigma > 0:
                    # Coincident points - distance is zero
                    return 0.0
                else:
                    # Antipodal or near-antipodal points - use haversine fallback
                    return self._haversine_distance(lat1, lon1, lat2, lon2)
            
            cos_sigma = sin_U1 * sin_U2 + cos_U1 * cos_U2 * cos_lam
            sigma = np.arctan2(sin_sigma, cos_sigma)
            
            sin_alpha = cos_U1 * cos_U2 * sin_lam / sin_sigma
            cos2_alpha = 1 - sin_alpha**2
            
            if cos2_alpha < 1e-12:
                cos_2sigma_m = 0.0
            else:
                cos_2sigma_m = cos_sigma - 2 * sin_U1 * sin_U2 / cos2_alpha
            
            C = f / 16 * cos2_alpha * (4 + f * (4 - 3 * cos2_alpha))
            
            lam_prev = lam
            lam = L + (1 - C) * f * sin_alpha * (
                sigma + C * sin_sigma * (cos_2sigma_m + C * cos_sigma * 
                                         (-1 + 2 * cos_2sigma_m**2)))
            
            if abs(lam - lam_prev) < 1e-12:
                converged = True
                break
        
        if not converged:
            warnings.warn(
                f"Vincenty's formula did not converge after 100 iterations. "
                f"Using haversine fallback.",
                RuntimeWarning
            )
            return self._haversine_distance(lat1, lon1, lat2, lon2)
        
        u2 = cos2_alpha * (a**2 - b**2) / b**2
        A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
        B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
        
        delta_sigma = B * sin_sigma * (
            cos_2sigma_m + B / 4 * (cos_sigma * (-1 + 2 * cos_2sigma_m**2) -
                                    B / 6 * cos_2sigma_m * (-3 + 4 * sin_sigma**2) *
                                    (-3 + 4 * cos_2sigma_m**2)))
        
        s = b * A * (sigma - delta_sigma)
        
        return s
    
    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        """Haversine distance (spherical fallback for Vincenty)."""
        R = 6371008.8  # Mean Earth radius in meters
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        
        return R * c
    
    def compute_gravity(self, pos_ned, mass, model="constant"):
        """Compute gravity force in NED (N) using selected model."""
        if model == "constant":
            return self.gravity_constant(mass)
        
        elif model == "somigliana":
            pos_ecef = self.ned_to_ecef(pos_ned)
            lat, _, alt = self.ecef_to_lla(pos_ecef[0], pos_ecef[1], pos_ecef[2])
            return self.gravity_somigliana(lat, alt, mass)
        
        elif model == "j2":
            pos_ecef = self.ned_to_ecef(pos_ned)
            return self.gravity_j2(pos_ecef, mass)
        
        elif model == "j2_j3_j4":
            pos_ecef = self.ned_to_ecef(pos_ned)
            return self.gravity_j2_j3_j4(pos_ecef, mass)
        
        else:
            return self.gravity_constant(mass)


class RocketDynamics:
    """6-DOF rigid body dynamics with Newton-Euler equations, time-varying mass/inertia.

    State: [x,y,z, vx,vy,vz, q0,q1,q2,q3, wx,wy,wz] (13 elements, NED frame).
    Gravity models: "constant", "somigliana", "j2", "j2_j3_j4".
    """
    
    def __init__(self, config, mass_properties_model=None):
        """
        Initialize rocket dynamics.
        
        Args:
            config: Configuration dictionary
            mass_properties_model: Optional MassPropertiesModel instance
        """
        
        self.use_mass_model = mass_properties_model is not None
        
        if self.use_mass_model:
            self.mass_model = mass_properties_model
        else:
            self.mass_dry = config['mass_dry']  # kg
            self.mass_propellant = config['mass_propellant']  # kg
            self.mass_total = self.mass_dry + self.mass_propellant
            
            # Support both nested 'inertia' dict and flat Ixx_0/Iyy_0/Izz_0 format
            if 'inertia' in config:
                self.Ixx = config['inertia']['Ixx']
                self.Iyy = config['inertia']['Iyy']
                self.Izz = config['inertia']['Izz']
            else:
                # Use initial (full propellant) inertia values
                self.Ixx = config.get('Ixx_0', config.get('Ixx', 1.0))
                self.Iyy = config.get('Iyy_0', config.get('Iyy', 1.0))
                self.Izz = config.get('Izz_0', config.get('Izz', 1.0))
            
            self.burn_time = config.get('burn_time', 5.0)
        
        self.g = 9.80665  # m/s^2
        
        earth_config = config.get('earth_model', {})
        self.gravity_model = earth_config.get('gravity_model', 'constant')
        
        origin_lla = None
        if 'origin_lla' in earth_config:
            origin_lla = earth_config['origin_lla']
        elif 'atmosphere' in config:
            lat_rad = config['atmosphere'].get('latitude_rad', 0.0)
            launch_alt = config['atmosphere'].get('launch_altitude_m', 0.0)
            origin_lla = [lat_rad, 0.0, launch_alt]
        
        self.earth_model = EarthModel(origin_lla=origin_lla)
        
        # inertia_derivative_max_norm: Maximum allowed ||İ|| (kg·m²/s)
        #   - None: Disable capping (use physical values, may cause instability with bad data)
        #   - float: Fixed cap value
        #   - 'auto': Scale-aware cap based on inertia magnitude and burn time
        self._inertia_derivative_max_norm = config.get('inertia_derivative_max_norm', 'auto')
        self._inertia_derivative_dt = config.get('inertia_derivative_dt', 0.001)

        # use_inertia_derivative: Include İ·ω term in Euler's equation (True) or ignore it (False)
        self._use_inertia_derivative = config.get('use_inertia_derivative', True)
    
    def get_mass(self, t):
        """Get current mass based on time."""
        if self.use_mass_model:
            return self.mass_model.get_mass(t)
        else:
            if t < self.burn_time:
                mass_consumed = (self.mass_propellant / self.burn_time) * t
                return self.mass_total - mass_consumed
            else:
                return self.mass_dry
    
    def get_inertia_matrix(self, t):
        """Get inertia matrix at time t."""
        if self.use_mass_model:
            return self.mass_model.get_inertia_matrix(t)
        else:
            return np.diag([self.Ixx, self.Iyy, self.Izz])
    
    def get_inertia_matrix_from_mass(self, current_mass: float):
        """Get inertia matrix based on current mass (for thrust scaling/error injection)."""
        if self.use_mass_model and hasattr(self.mass_model, 'get_inertia_matrix_from_mass'):
            return self.mass_model.get_inertia_matrix_from_mass(current_mass)
        else:
            # Legacy mode: constant inertia
            return np.diag([self.Ixx, self.Iyy, self.Izz])
    
    def get_cg_location(self, t):
        """Get center of gravity location at time t."""
        if self.use_mass_model:
            return self.mass_model.get_cg_location(t)
        else:
            return 0.0
    
    def get_mass_flow_rate(self, t):
        """
        Get mass flow rate at time t.
        
        This method provides a unified source of truth for mass flow rate,
        ensuring consistency between mass integration and inertia/CG calculations.
        
        Args:
            t: Time (s)
            
        Returns:
            Mass flow rate (kg/s)
        """
        if self.use_mass_model:
            return self.mass_model.get_mass_flow_rate(t)
        else:
            # Constant mass flow rate for legacy mode
            if t < self.burn_time:
                return self.mass_propellant / self.burn_time
            return 0.0
    
    def get_inertia_derivative(self, t, dt=None):
        """Compute İ (inertia derivative) via finite difference. Returns zero near separation/burnout."""
        if not self.use_mass_model:
            return np.zeros((3, 3))
        
        # Use configured dt if not provided
        if dt is None:
            dt = self._inertia_derivative_dt
        
        # Safeguard: Check if we're near a stage separation event or burnout event
        transition_epsilon = 2.0 * dt

        # Check separation times
        if hasattr(self.mass_model, 'separation_times'):
            for sep_time in self.mass_model.separation_times:
                if abs(t - sep_time) < transition_epsilon:
                    return np.zeros((3, 3))

        # Check burnout times - when thrust drops below 10% of max
        # This prevents numerical noise in İ calculation at burnout
        if hasattr(self.mass_model, 'burnout_times'):
            for burnout_time in self.mass_model.burnout_times:
                if abs(t - burnout_time) < transition_epsilon:
                    return np.zeros((3, 3))
        elif hasattr(self.mass_model, 'get_burnout_time'):
            burnout_time = self.mass_model.get_burnout_time()
            if burnout_time is not None and abs(t - burnout_time) < transition_epsilon:
                return np.zeros((3, 3))

        # Check depletion times - when propellant is fully consumed and mass stops changing
        # This is critical because İ transitions from non-zero to zero at depletion
        if hasattr(self.mass_model, 'depletion_times'):
            for depletion_time in self.mass_model.depletion_times:
                if abs(t - depletion_time) < transition_epsilon:
                    return np.zeros((3, 3))
        
        # Use use_cache=False to bypass cache for finite difference
        I_plus = self.mass_model.get_inertia_matrix(t + dt, use_cache=False)
        I_minus = self.mass_model.get_inertia_matrix(t - dt, use_cache=False)
        
        I_dot = (I_plus - I_minus) / (2.0 * dt)        
        max_norm = self._inertia_derivative_max_norm
        
        if max_norm is None:
            # Capping disabled - use physical values
            return I_dot
        
        if max_norm == 'auto':
            # Scale-aware cap: max_norm = factor * ||I|| / tau
            # where tau is a characteristic time scale (burn time or minimum 0.1s)
            I_current = self.mass_model.get_inertia_matrix(t)
            I_scale = np.linalg.norm(I_current)
            
            # Get burn time from current stage if available
            tau = 0.1  # Default minimum time scale
            if hasattr(self.mass_model, 'get_current_stage'):
                stage = self.mass_model.get_current_stage()
                if hasattr(stage, 'burn_time') and stage.burn_time > 0:
                    tau = max(stage.burn_time, 0.1)
            
            # Factor of 10 allows for reasonable variation while catching anomalies
            max_norm = 10.0 * I_scale / tau
        
        I_dot_magnitude = np.linalg.norm(I_dot)
        if I_dot_magnitude > max_norm:
            warnings.warn(
                f"İ magnitude ({I_dot_magnitude:.2e} kg·m²/s) exceeds limit ({max_norm:.2e} kg·m²/s) "
                f"at t={t:.4f}s. Capping to limit. This may indicate bad input data or "
                f"numerical issues. Consider adjusting 'inertia_derivative_max_norm' config.",
                RuntimeWarning
            )
            I_dot = I_dot * (max_norm / I_dot_magnitude)
        
        return I_dot
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to 3x3 rotation matrix (delegates to quaternion_utils)."""
        return _quaternion_to_rotation_matrix(q)
    
    def quaternion_derivative(self, q, omega):
        """Compute quaternion derivative: dq/dt = 0.5 × Ω(ω) × q."""
        q = np.asarray(q)
        omega = np.asarray(omega)

        # Check for NaN or Inf values in quaternion
        if not np.all(np.isfinite(q)):
            raise DegenerateQuaternionError(
                f"Quaternion contains NaN or Inf values: {q}. "
                f"Cannot compute quaternion derivative."
            )

        # Check for NaN or Inf values in angular velocity
        if not np.all(np.isfinite(omega)):
            raise DegenerateQuaternionError(
                f"Angular velocity contains NaN or Inf values: {omega}. "
                f"Cannot compute quaternion derivative."
            )
        
        q0, q1, q2, q3 = q
        wx, wy, wz = omega
        
        Omega = 0.5 * np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])
        
        q_dot = Omega @ q
        
        return q_dot
    
    def calculate_derivatives(self, position, velocity, quaternion, angular_velocity,
                            forces_body, moments_body, mass, t=0.0,
                            use_mass_based_inertia=False, mass_flow=0.0, dry_mass=0.0,
                            xbc=0.0):
        """Calculate 13-element state derivatives (position, velocity, quaternion, angular velocity)."""
        # Note: Quaternion normalization is handled by _normalize_state after each integration step
        # Normalizing here would affect integrator accuracy (RK4 calls derivatives multiple times per step)
        
        R = self.quaternion_to_rotation_matrix(quaternion)
        
        gravity_ned = self.earth_model.compute_gravity(position, mass, self.gravity_model)
        
        forces_ned = R @ forces_body
        
        total_forces_ned = forces_ned + gravity_ned
        
        acceleration_ned = total_forces_ned / mass
        
        position_dot = velocity
        
        velocity_dot = acceleration_ned
        
        quaternion_dot = self.quaternion_derivative(quaternion, angular_velocity)
        
        # Use mass-based inertia when thrust scaling or error injection is active
        if use_mass_based_inertia:
            I = self.get_inertia_matrix_from_mass(mass)
            # Approximate I_dot using finite difference on mass
            dm = 0.01 * mass if mass > 0 else 0.01
            if mass_flow > 0 and mass > dry_mass + dm:
                I_plus = self.get_inertia_matrix_from_mass(mass - dm)
                I_minus = self.get_inertia_matrix_from_mass(mass + dm)
                dI_dm = (I_minus - I_plus) / (2 * dm)
                I_dot = dI_dm * (-mass_flow)
            else:
                I_dot = np.zeros((3, 3))
        else:
            I = self.get_inertia_matrix(t)
            I_dot = self.get_inertia_derivative(t)
        
        I_omega = I @ angular_velocity
        omega_cross_I_omega = np.cross(angular_velocity, I_omega)
        
        # Conditionally include İ·ω term based on configuration
        if self._use_inertia_derivative:
            I_dot_omega = I_dot @ angular_velocity
        else:
            # Simplified Euler equation: ignore İ·ω term
            I_dot_omega = np.zeros(3)

        # CG offset moment correction
        M_cg_offset = np.array([
            0.0,                    # Roll moment (no contribution from xbc)
            xbc * forces_body[2],   # Pitch moment: xbc × F_normal
            -xbc * forces_body[1]   # Yaw moment: -xbc × F_side
        ])

        # Total moments including CG offset correction
        total_moments = moments_body + M_cg_offset

        # Euler equation: ω̇ = I⁻¹·(M - ω×(I·ω) - İ·ω)
        angular_acceleration = np.linalg.solve(I, total_moments - omega_cross_I_omega - I_dot_omega)
        
        derivatives = np.concatenate([
            position_dot,
            velocity_dot,
            quaternion_dot,
            angular_acceleration
        ])
        
        return derivatives
