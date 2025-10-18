#!/bin/bash

# ============================================================================
# ChesterTheBus RV Control - Build All Projects & Populate Cache
# ============================================================================
# This script builds all projects to populate the PlatformIO and VS Code cache
# Run this once to speed up future builds significantly!
# ============================================================================

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║     Building All ChesterTheBus Projects - Populating Cache   ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Counter for successful builds
SUCCESS_COUNT=0
FAIL_COUNT=0
TOTAL_PROJECTS=6

# Array of project directories
PROJECTS=(
    "Mopeka Testing"
    "GIGA_R1_TheBrain"
    "D1 Plumbing system"
    "D1Mini_CabLock"
    "D1Mini_CabLock_KitchenDriverSide"
    "D1Mini_CabLock_KitchenPassSide"
)

echo "Found $TOTAL_PROJECTS projects to build:"
for PROJECT in "${PROJECTS[@]}"; do
    echo "  • $PROJECT"
done
echo ""
echo "This will take 10-20 minutes on first run (downloading libraries)"
echo "But future builds will be 10-30 seconds! ⚡"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Build cancelled."
    exit 0
fi

echo ""
echo "Starting builds..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Build each project
for i in "${!PROJECTS[@]}"; do
    PROJECT="${PROJECTS[$i]}"
    PROJECT_NUM=$((i + 1))

    echo "[$PROJECT_NUM/$TOTAL_PROJECTS] Building: $PROJECT"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    if [ -d "$PROJECT" ]; then
        cd "$PROJECT"

        # Check if platformio.ini exists
        if [ -f "platformio.ini" ]; then
            echo "  → Compiling $PROJECT..."

            # Try to build using platformio command from VS Code extension
            # This works even if 'pio' isn't in PATH
            if command -v pio &> /dev/null; then
                pio run
            elif command -v platformio &> /dev/null; then
                platformio run
            else
                # Use Python module directly
                python3 -m platformio run 2>/dev/null || ~/.platformio/python3/bin/python3 -m platformio run 2>/dev/null
            fi

            BUILD_RESULT=$?

            if [ $BUILD_RESULT -eq 0 ]; then
                echo "  ✅ $PROJECT built successfully!"
                SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
            else
                echo "  ❌ $PROJECT build failed (code: $BUILD_RESULT)"
                FAIL_COUNT=$((FAIL_COUNT + 1))
            fi
        else
            echo "  ⚠️  No platformio.ini found in $PROJECT"
            FAIL_COUNT=$((FAIL_COUNT + 1))
        fi

        cd "$SCRIPT_DIR"
    else
        echo "  ⚠️  Directory not found: $PROJECT"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi

    echo ""
done

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║                    BUILD SUMMARY                             ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "  ✅ Successful builds: $SUCCESS_COUNT"
echo "  ❌ Failed builds:     $FAIL_COUNT"
echo "  📊 Total projects:    $TOTAL_PROJECTS"
echo ""

if [ $SUCCESS_COUNT -gt 0 ]; then
    echo "🎉 Cache has been populated for $SUCCESS_COUNT project(s)!"
    echo ""
    echo "📂 Cache locations:"
    echo "  • Each project's .pio/ folder (build artifacts)"
    echo "  • ~/.platformio/ (shared libraries)"
    echo "  • .vscode/.cache/ (IntelliSense)"
    echo ""
    echo "⚡ Benefits:"
    echo "  • Rebuilds now take 10-30 seconds (vs 3-5 minutes!)"
    echo "  • IntelliSense is now instant"
    echo "  • Code completion is real-time"
    echo ""
fi

if [ $FAIL_COUNT -gt 0 ]; then
    echo "⚠️  $FAIL_COUNT project(s) failed to build."
    echo ""
    echo "Common issues:"
    echo "  • PlatformIO not installed: Use VS Code to build instead"
    echo "  • Missing dependencies: Check platformio.ini"
    echo "  • Network issues: Libraries may not have downloaded"
    echo ""
    echo "Solution: Open VS Code and use Command Palette:"
    echo "  Cmd+Shift+P → 'PlatformIO: Build'"
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "🚌 ChesterTheBus RV Control - Cache Ready!"
echo ""
