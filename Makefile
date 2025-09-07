.PHONY: help build test clean install dev lint format check-env

PACKAGE_NAME = ros2_systemd
PYTHON = python3
COLCON = colcon
ROS_DISTRO ?= humble

help:
	@echo "Available targets:"
	@echo "  help       - Show this help message"
	@echo "  build      - Build the package with colcon"
	@echo "  test       - Run tests"
	@echo "  clean      - Remove build artifacts"
	@echo "  install    - Install the package locally"
	@echo "  dev        - Setup development environment"
	@echo "  lint       - Run code linters"
	@echo "  format     - Format code with black"

check-env:
	@if [ -z "$(ROS_DISTRO)" ]; then \
		echo "Error: ROS2 environment not sourced"; \
		echo "Run: source /opt/ros/$(ROS_DISTRO)/setup.bash"; \
		exit 1; \
	fi

build: check-env
	@echo "Building $(PACKAGE_NAME)..."
	@$(COLCON) build --symlink-install --packages-select $(PACKAGE_NAME)
	@echo "Build complete. Run 'source install/setup.bash' to use."

test: check-env
	@echo "Running tests..."
	@if [ -f install/setup.bash ]; then \
		. install/setup.bash && $(COLCON) test --packages-select $(PACKAGE_NAME); \
		$(COLCON) test-result --verbose; \
	else \
		echo "Running local tests without build..."; \
		$(PYTHON) test_manual.py; \
	fi

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf build install log
	@rm -rf __pycache__ */__pycache__ */*/__pycache__
	@rm -f ros2_systemd_dev
	@find . -name "*.pyc" -delete
	@find . -name "*.pyo" -delete
	@find . -name ".pytest_cache" -type d -exec rm -rf {} + 2>/dev/null || true
	@echo "Clean complete."

install: build
	@echo "Installing $(PACKAGE_NAME)..."
	@. install/setup.bash && $(COLCON) build --symlink-install --packages-select $(PACKAGE_NAME)
	@echo "Installation complete."

dev:
	@echo "Setting up development environment..."
	@chmod +x dev_setup.sh test_manual.py test_local.sh
	@./dev_setup.sh
	@echo "Development environment ready."

lint:
	@echo "Running linters..."
	@$(PYTHON) -m flake8 $(PACKAGE_NAME) --max-line-length=120 --exclude=__pycache__ 2>/dev/null || \
		(echo "flake8 not installed. Install with: pip3 install flake8" && exit 1)
	@$(PYTHON) -m pylint $(PACKAGE_NAME) --max-line-length=120 2>/dev/null || \
		(echo "pylint not installed. Install with: pip3 install pylint" && exit 0)

format:
	@echo "Formatting code..."
	@$(PYTHON) -m black $(PACKAGE_NAME) --line-length=120 2>/dev/null || \
		(echo "black not installed. Install with: pip3 install black" && exit 1)
	@$(PYTHON) -m isort $(PACKAGE_NAME) 2>/dev/null || \
		(echo "isort not installed. Install with: pip3 install isort" && exit 0)
