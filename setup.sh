#!/bin/bash
# DDCL Project - Quick Setup Script
# This script helps set up the SUMO environment for the DDCL simulation

set -e  # Exit on error

echo "======================================================================"
echo "DDCL - Dynamic CAV Dedicated Lane Management System"
echo "Quick Setup Script"
echo "======================================================================"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "  $1"
}

# Step 1: Check Python
echo "Step 1: Checking Python installation..."
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version 2>&1)
    print_success "Python found: $PYTHON_VERSION"
else
    print_error "Python 3 not found. Please install Python 3.7 or later."
    exit 1
fi
echo ""

# Step 2: Check pip
echo "Step 2: Checking pip installation..."
if command -v pip3 &> /dev/null; then
    PIP_VERSION=$(pip3 --version 2>&1)
    print_success "pip found: $PIP_VERSION"
else
    print_warning "pip3 not found. Will try pip..."
    if command -v pip &> /dev/null; then
        print_success "pip found"
    else
        print_error "pip not found. Please install pip."
        exit 1
    fi
fi
echo ""

# Step 3: Install Python dependencies
echo "Step 3: Installing Python dependencies..."
print_info "Installing: traci, numpy, pandas, matplotlib"

# Check if running in virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    print_warning "Not in a virtual environment. Installing globally (may require sudo)."
    read -p "Continue? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Setup cancelled. Consider using a virtual environment:"
        print_info "  python3 -m venv venv"
        print_info "  source venv/bin/activate"
        print_info "  ./setup.sh"
        exit 1
    fi
fi

# Try to install dependencies
if pip3 install traci numpy pandas matplotlib jupyter 2>&1; then
    print_success "Python dependencies installed"
else
    print_warning "Installation with pip3 failed, trying pip..."
    if pip install traci numpy pandas matplotlib jupyter 2>&1; then
        print_success "Python dependencies installed"
    else
        print_error "Failed to install Python dependencies"
        exit 1
    fi
fi
echo ""

# Step 4: Check SUMO
echo "Step 4: Checking SUMO installation..."
if command -v sumo &> /dev/null; then
    SUMO_VERSION=$(sumo --version 2>&1 | head -1)
    print_success "SUMO found: $SUMO_VERSION"
    
    # Check SUMO_HOME
    if [ -z "$SUMO_HOME" ]; then
        print_warning "SUMO_HOME environment variable not set"
        
        # Try to find SUMO installation
        SUMO_PATH=$(which sumo)
        SUMO_BIN_DIR=$(dirname "$SUMO_PATH")
        SUMO_HOME_GUESS=$(dirname "$SUMO_BIN_DIR")
        
        print_info "Detected SUMO at: $SUMO_HOME_GUESS"
        print_info "Add to your ~/.bashrc or ~/.zshrc:"
        echo ""
        echo "    export SUMO_HOME=$SUMO_HOME_GUESS"
        echo ""
        
        # Set for current session
        export SUMO_HOME="$SUMO_HOME_GUESS"
        print_success "SUMO_HOME set for current session: $SUMO_HOME"
    else
        print_success "SUMO_HOME is set: $SUMO_HOME"
    fi
else
    print_error "SUMO not found. Please install SUMO:"
    print_info ""
    print_info "Ubuntu/Debian:"
    print_info "  sudo add-apt-repository ppa:sumo/stable"
    print_info "  sudo apt-get update"
    print_info "  sudo apt-get install sumo sumo-tools sumo-doc"
    print_info ""
    print_info "macOS (with Homebrew):"
    print_info "  brew install sumo"
    print_info ""
    print_info "Windows:"
    print_info "  Download from: https://sumo.dlr.de/docs/Downloads.php"
    exit 1
fi
echo ""

# Step 5: Validate configuration
echo "Step 5: Validating SUMO configuration files..."
if python3 validate_sumo_config.py; then
    print_success "SUMO configuration is valid"
else
    print_error "SUMO configuration validation failed"
    exit 1
fi
echo ""

# Step 6: Summary
echo "======================================================================"
echo "Setup Complete!"
echo "======================================================================"
echo ""
print_success "All dependencies are installed and configured"
echo ""
echo "Next steps:"
echo ""
echo "1. Run the simulation with Python controller:"
echo "   cd V1.6/controller"
echo "   python3 main.py"
echo ""
echo "2. Or test SUMO directly with GUI:"
echo "   cd V1.6/scenario"
echo "   sumo-gui -c test.sumocfg"
echo ""
echo "3. Analyze results:"
echo "   cd V1.6/analysis"
echo "   jupyter notebook run_comparison.ipynb"
echo ""
echo "For more information, see README.md"
echo ""
