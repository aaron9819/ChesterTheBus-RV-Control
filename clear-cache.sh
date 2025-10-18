#!/bin/bash

# ============================================================================
# ChesterTheBus RV Control - Cache Management Script
# ============================================================================
# This script helps manage build caches and temporary files
# Usage:
#   ./clear-cache.sh        - Clear all caches
#   ./clear-cache.sh soft   - Clear only build artifacts (keep libraries)
#   ./clear-cache.sh pio    - Clear only PlatformIO cache
#   ./clear-cache.sh vscode - Clear only VS Code cache
# ============================================================================

echo "============================================"
echo "ChesterTheBus RV Control - Cache Manager"
echo "============================================"
echo ""

# Determine what to clear
CLEAR_MODE="${1:-all}"

case "$CLEAR_MODE" in
  soft)
    echo "🧹 Soft clean: Clearing build artifacts only..."
    find . -name ".pio/build" -type d -exec rm -rf {} + 2>/dev/null
    echo "✓ Build artifacts cleared"
    ;;

  pio)
    echo "🧹 Clearing PlatformIO cache..."
    find . -name ".pio" -type d -exec rm -rf {} + 2>/dev/null
    echo "✓ PlatformIO cache cleared"
    ;;

  vscode)
    echo "🧹 Clearing VS Code cache..."
    find . -path "*/.vscode/.cache" -type d -exec rm -rf {} + 2>/dev/null
    find . -name ".vscode/.browse.c_cpp.db*" -exec rm -f {} + 2>/dev/null
    find . -name ".vscode/ipch" -type d -exec rm -rf {} + 2>/dev/null
    echo "✓ VS Code cache cleared"
    ;;

  all)
    echo "🧹 Full clean: Clearing all caches..."

    # PlatformIO caches
    echo "  → Clearing PlatformIO builds..."
    find . -name ".pio" -type d -exec rm -rf {} + 2>/dev/null

    # VS Code caches
    echo "  → Clearing VS Code caches..."
    find . -path "*/.vscode/.cache" -type d -exec rm -rf {} + 2>/dev/null
    find . -name ".vscode/.browse.c_cpp.db*" -exec rm -f {} + 2>/dev/null
    find . -name ".vscode/ipch" -type d -exec rm -rf {} + 2>/dev/null

    # General caches
    echo "  → Clearing general caches..."
    find . -name ".cache" -type d -exec rm -rf {} + 2>/dev/null
    find . -name "*.cache" -type f -exec rm -f {} + 2>/dev/null

    # macOS specific
    echo "  → Clearing macOS files..."
    find . -name ".DS_Store" -exec rm -f {} + 2>/dev/null

    # Backup files
    echo "  → Clearing backup files..."
    find . -name "*.bak" -exec rm -f {} + 2>/dev/null
    find . -name "*~" -exec rm -f {} + 2>/dev/null

    echo "✓ All caches cleared"
    ;;

  *)
    echo "❌ Unknown option: $CLEAR_MODE"
    echo ""
    echo "Usage:"
    echo "  ./clear-cache.sh        - Clear all caches"
    echo "  ./clear-cache.sh soft   - Clear only build artifacts"
    echo "  ./clear-cache.sh pio    - Clear only PlatformIO cache"
    echo "  ./clear-cache.sh vscode - Clear only VS Code cache"
    exit 1
    ;;
esac

echo ""
echo "🎉 Cache management complete!"
echo ""
echo "Projects in workspace:"
echo "  • GIGA_R1_TheBrain (Main Controller)"
echo "  • D1 Plumbing system (Temperature & Safety)"
echo "  • D1Mini_CabLock variants (Cabinet Locks)"
echo "  • Mopeka Testing (Tank Level Sensors)"
echo ""
echo "To rebuild a project:"
echo "  cd <project-folder>"
echo "  pio run"
echo ""
echo "============================================"
