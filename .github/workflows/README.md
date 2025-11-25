# GitHub Actions Workflows

## Compile Arduino Sketches (`compile.yml`)

Compiles both Arduino Uno and ATTiny85 firmware and generates compiled `.hex` files.

### What it does
1. Installs Arduino CLI and required cores
2. Compiles Arduino Uno sketch → `arduino_uno/compiled/`
3. Compiles ATTiny85 sketch → `attiny85/compiled/`
4. Uploads compiled firmware as artifacts
5. Displays memory usage statistics

### Output
- **Compiled firmware files**: `.hex`, `.elf`, and other build artifacts
- **Workflow artifacts**: `compiled-firmware` containing both versions

## Create Repository ZIP (`create-zip.yml`)

Creates a ZIP archive with the naming format: `ddmmyyyy_reponame.zip` including source code and compiled firmware.

### Dependencies
- **Requires**: Compilation workflow must run first to generate firmware artifacts
- **Automatic**: Downloads latest compiled firmware from compilation workflow

### Example Output
- `25112025_PID_simple_controll.zip` (November 25, 2025)

### ZIP Contents
- ✅ **Source code**: All Arduino sketches, configs, and documentation
- ✅ **Compiled firmware**: `.hex` files for both Arduino Uno and ATTiny85
- ❌ **Excluded**: `.git` directories, workflow files, development artifacts

### Triggers
- **Manual**: Go to Actions → Create Repository ZIP → Run workflow
- **Release**: Automatically runs when a release is published
- **Push**: Runs on every push to main/master branches

### Workflow Sequence
1. **Compilation workflow** runs first (compiles firmware)
2. **ZIP workflow** runs second (downloads firmware + creates archive)
3. **Result**: Complete package with source + compiled firmware

### Usage
- **Manual ZIP creation**: Use workflow dispatch to create ZIP anytime
- **Release artifacts**: Automatic ZIP creation for releases with compiled firmware
- **Continuous deployment**: ZIP created on every main branch update

### Output Location
- **Workflow artifacts**: Download from Actions → Create Repository ZIP → Artifacts
- **Release assets**: Automatically attached to GitHub releases

### File Structure in ZIP
```
25112025_PID_simple_controll.zip/
├── arduino_uno/
│   ├── arduino_uno.ino
│   ├── config.h
│   ├── compiled/          # ← Compiled firmware
│   │   ├── arduino_uno.ino.hex
│   │   └── arduino_uno.ino.elf
│   └── ...
├── attiny85/
│   ├── attiny85.ino
│   ├── config.h
│   ├── compiled/          # ← Compiled firmware
│   │   ├── attiny85.ino.hex
│   │   └── attiny85.ino.elf
│   └── ...
├── assets/
├── README.md
└── ...
```
