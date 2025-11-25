# GitHub Actions Workflows

## Compile Arduino Sketches (`compile.yml`) - **PRIMARY WORKFLOW**

**This is the main workflow that does everything!** Compiles firmware and creates complete ZIP packages.

### What it does
1. Installs Arduino CLI and required cores
2. Compiles Arduino Uno sketch → `arduino_uno/compiled/`
3. Compiles ATTiny85 sketch → `attiny85/compiled/`
4. Creates complete ZIP with source + compiled firmware
5. Uploads ZIP as artifact and attaches to releases
6. Displays memory usage statistics

### Example Output
- `25112025_PID_simple_controll.zip` (November 25, 2025)

### ZIP Contents (Complete Package)
- ✅ **Source code**: All Arduino sketches, configs, and documentation
- ✅ **Compiled firmware**: Ready-to-flash `.hex` files for both platforms
- ✅ **Documentation**: Complete setup guides and troubleshooting
- ❌ **Excluded**: `.git` directories, workflow files

### Triggers
- **Push**: Runs on every push to main/master branches
- **Pull Request**: Runs on PRs to check compilation
- **Release**: Creates ZIP and attaches to GitHub releases
- **Manual**: Actions → "Compile Arduino Sketches" → Run workflow

### Output Locations
- **ZIP Artifacts**: Actions → Compile Arduino Sketches → `complete-package`
- **Release Assets**: Automatically attached to GitHub releases
- **Memory Reports**: Shows flash/RAM usage in workflow logs

### File Structure in ZIP
```
25112025_PID_simple_controll.zip/
├── arduino_uno/
│   ├── arduino_uno.ino        # Source
│   ├── config.h               # Config
│   ├── compiled/              # ← Firmware ready to flash
│   │   ├── arduino_uno.ino.hex
│   │   └── arduino_uno.ino.elf
│   └── README.md
├── attiny85/
│   ├── attiny85.ino           # Source
│   ├── config.h               # Config
│   ├── compiled/              # ← Firmware ready to flash
│   │   ├── attiny85.ino.hex
│   │   └── attiny85.ino.elf
│   └── README.md
├── assets/                    # Documentation
├── README.md                  # Main docs
└── ...
```

## Create Source Code ZIP (`create-zip.yml`) - **SECONDARY WORKFLOW**

Creates a ZIP of only source code (no compiled firmware) for special cases.

### When to use
- When you want just source code without binaries
- For code review or development purposes
- When compiled firmware isn't needed

### Output
- `25112025_PID_simple_controll_source.zip` (source only)

### Triggers
- **Manual Only**: Actions → "Create Source Code ZIP" → Run workflow

---

## Workflow Summary

| Workflow | Purpose | Output | Use Case |
|----------|---------|---------|----------|
| **Compile Arduino Sketches** | Full build + package | Complete ZIP with firmware | **Primary - Use this!** |
| **Create Source Code ZIP** | Source only | Source code ZIP | Special cases only |

**For most users: Just use "Compile Arduino Sketches" - it does everything!** 🚀
