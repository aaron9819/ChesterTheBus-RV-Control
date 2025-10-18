#!/bin/bash

# ============================================================================
# ChesterTheBus RV Control - Environment Setup Script
# ============================================================================
# This script helps you set up environment variables for development
# Run this once to configure your development environment
# ============================================================================

echo "============================================"
echo "ChesterTheBus RV Control - Environment Setup"
echo "============================================"
echo ""


# Detect shell
SHELL_CONFIG=""
if [ -f "$HOME/.zshrc" ]; then
    SHELL_CONFIG="$HOME/.zshrc"
    SHELL_NAME="zsh"
elif [ -f "$HOME/.bashrc" ]; then
    SHELL_CONFIG="$HOME/.bashrc"
    SHELL_NAME="bash"
else
    echo "⚠️  Could not detect shell config file"
    echo "Please manually add environment variables to your shell config"
    exit 1
fi

echo "Detected shell: $SHELL_NAME"
echo "Config file: $SHELL_CONFIG"
echo ""

# Check if Anthropic API key is already set
if grep -q "ANTHROPIC_API_KEY" "$SHELL_CONFIG"; then
    echo "✓ Anthropic API key already configured in $SHELL_CONFIG"
else
    echo "📝 Setting up Anthropic API key..."
    echo "" >> "$SHELL_CONFIG"
    echo "# ChesterTheBus RV Control - API Keys" >> "$SHELL_CONFIG"
    echo "export ANTHROPIC_API_KEY=\"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\"" >> "$SHELL_CONFIG"
    echo "✓ Anthropic API key added to $SHELL_CONFIG"
fi

# Set up PlatformIO cache directory
if grep -q "PLATFORMIO_CACHE_DIR" "$SHELL_CONFIG"; then
    echo "✓ PlatformIO cache directory already configured"
else
    echo "📝 Setting up PlatformIO cache directory..."
    echo "export PLATFORMIO_CACHE_DIR=\"$HOME/.platformio/.cache\"" >> "$SHELL_CONFIG"
    echo "✓ PlatformIO cache directory configured"
fi

# Optional: Set up ChesterTheBus project path
PROJECT_PATH="$(cd "$(dirname "$0")" && pwd)"
if grep -q "CHESTER_PROJECT_PATH" "$SHELL_CONFIG"; then
    echo "✓ ChesterTheBus project path already configured"
else
    echo "📝 Setting up ChesterTheBus project path..."
    echo "export CHESTER_PROJECT_PATH=\"$PROJECT_PATH\"" >> "$SHELL_CONFIG"
    echo "✓ Project path configured: $PROJECT_PATH"
fi

echo ""
echo "============================================"
echo "✅ Environment setup complete!"
echo "============================================"
echo ""
echo "To activate these settings, run:"
echo "  source $SHELL_CONFIG"
echo ""
echo "Or restart your terminal."
echo ""
echo "Configured environment variables:"
echo "  • ANTHROPIC_API_KEY    - For AI features"
echo "  • PLATFORMIO_CACHE_DIR - For faster builds"
echo "  • CHESTER_PROJECT_PATH - Project location"
echo ""
echo "Security Note:"
echo "Your API key is stored in $SHELL_CONFIG"
echo "Make sure this file is NOT committed to git!"
echo ""
echo "To verify setup:"
echo "  echo \$ANTHROPIC_API_KEY"
echo ""
echo "============================================"
