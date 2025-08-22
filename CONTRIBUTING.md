# Contributing

Thanks for your interest in improving this driver!

## Code style
- C99
- No dynamic allocation
- Return codes: 0 = success, negative = error (`BQ25180_ERR_*`)
- Prefer explicit units in public APIs (mV, mA)
- Keep internal helpers `static`
- Add concise comments mapping fields to datasheet bit names

## Commit messages
- Use clear, imperative subjects (e.g., "Add TS HOT/COLD setters")
- Reference files/APIs touched when helpful

## Tests
- Add or update host-only unit tests in `tests/` using the fake I2C device
- Tests should print expectations/results when meaningful
- Ensure `ctest` passes on all supported hosts

## CI
- GitHub Actions runs on Windows/Linux/macOS for PRs and pushes

## Opening Issues/PRs
- Describe the motivation and behavior change
- For register/bitfield updates, include datasheet section/table references
- If behavior varies by datasheet revision, document the assumption and proposed handling
