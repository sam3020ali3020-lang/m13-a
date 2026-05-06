# Memory Profile — `smoketest`

- **Duration:** 0.1 min
- **Samples:** 4
- **Verdict:** 🔴 LEAK — significant growth in native heap

## Slope (linear regression)

| Metric | Start (KB) | End (KB) | Δ (KB) | Slope (KB/min) |
|--------|-----------:|---------:|-------:|---------------:|
| Native PSS | 22278 | 22282 | +4 | +24.0 |
| Native Heap Alloc | 27565 | 27120 | -445 | -2662.0 |
| Dalvik PSS | 9386 | 4482 | -4904 | -29680.0 |
| Dalvik Heap Alloc | 3981 | 3836 | -145 | -870.0 |
| Total PSS | 115063 | 110175 | -4888 | -29600.0 |

## Samples

| t(min) | Native PSS | Native Alloc | Dalvik PSS | Total PSS | Views | Activities |
|-------:|-----------:|-------------:|-----------:|----------:|------:|-----------:|
| 0.0 | 22278 | 27565 | 9386 | 115063 | 8 | 1 |
| 0.1 | 22282 | 27113 | 4610 | 110299 | 8 | 1 |
| 0.1 | 22282 | 27117 | 4482 | 110163 | 8 | 1 |
| 0.1 | 22282 | 27120 | 4482 | 110175 | 8 | 1 |
