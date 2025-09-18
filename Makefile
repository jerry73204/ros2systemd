PYTHON = python3
COLCON = colcon
ROS_DISTRO ?= humble

# Build flags
BUILD_FLAGS = --symlink-install

# Test flags
TEST_FLAGS =

.PHONY: help
help:
	@echo "Available targets:"
	@echo "  help       - Show this help message"
	@echo "  deps       - Install dependencies using rosdep"
	@echo "  build      - Build packages with colcon"
	@echo "  test       - Run tests for packages"
	@echo "  clean      - Remove build artifacts"
	@echo "  lint       - Run flake8 code linter"
	@echo "  format     - Format code with black"
	@echo "  check-style - Check code style with isort and pylint"
	@echo "  wheel      - Build a Python wheel for distribution"
	@echo "  release    - Build wheel and prepare for release"

.PHONY: check-env
check-env:
	@if [ -z "$(ROS_DISTRO)" ]; then \
		echo "Error: ROS2 environment not sourced"; \
		echo "Run: source /opt/ros/$(ROS_DISTRO)/setup.bash"; \
		exit 1; \
	fi

.PHONY: deps
deps: check-env
	@echo "Installing dependencies with rosdep..."
	@echo "This will install all dependencies listed in package.xml files."
	@. /opt/ros/$(ROS_DISTRO)/setup.sh && rosdep install --from-paths . --ignore-src -r -y --rosdistro $(ROS_DISTRO)
	@echo "Dependencies installed successfully."

.PHONY: build
build: check-env
	@echo "Building packages..."
	@$(COLCON) build $(BUILD_FLAGS)
	@echo "Build complete. Run 'source install/setup.bash' to use."

.PHONY: test
test: check-env
	@echo "Running tests..."
	@. /opt/ros/$(ROS_DISTRO)/setup.sh && \
	$(COLCON) test $(TEST_FLAGS)
	@. /opt/ros/$(ROS_DISTRO)/setup.sh && \
	$(COLCON) test-result --verbose

.PHONY: test-coverage
test-coverage: check-env
	@echo "Running tests with coverage..."
	@. /opt/ros/$(ROS_DISTRO)/setup.sh && \
	$(COLCON) test $(TEST_FLAGS) --pytest-with-coverage
	@. /opt/ros/$(ROS_DISTRO)/setup.sh && \
	$(COLCON) test-result --verbose

.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf build install log
	@rm -rf __pycache__ */__pycache__ */*/__pycache__
	@rm -f ros2systemd_dev
	@rm -rf dist *.egg-info ros2systemd.egg-info
	@find . -name "*.pyc" -delete
	@find . -name "*.pyo" -delete
	@find . -name ".pytest_cache" -type d -exec rm -rf {} + 2>/dev/null || true
	@echo "Clean complete."

.PHONY: lint
lint: check-env
	@echo "Running linters..."
	@if ! command -v flake8 >/dev/null 2>&1; then \
		echo "Error: flake8 not found."; \
		echo "Install dependencies with: make deps"; \
		echo "Or manually: sudo apt install flake8"; \
		exit 1; \
	fi
	@$(PYTHON) -m flake8 ros2systemd --max-line-length=120 \
		--exclude=__pycache__ \
		--extend-ignore=Q000,D100,D101,D102,D103,D104,D105,D107

.PHONY: format
format: check-env
	@echo "Formatting code..."
	@if ! command -v black >/dev/null 2>&1; then \
		echo "Error: black not found."; \
		echo "Install dependencies with: make deps"; \
		echo "Or manually: sudo apt install black"; \
		exit 1; \
	fi
	@$(PYTHON) -m black ros2systemd tests --line-length=120

.PHONY: check-style
check-style: check-env
	@echo "Checking code style with isort and pylint (optional tools)..."
	@if ! command -v isort >/dev/null 2>&1; then \
		echo "Note: isort not found. Install with: sudo apt install python3-isort"; \
	else \
		echo "Checking import order with isort..."; \
		$(PYTHON) -m isort ros2systemd tests --check-only --diff; \
	fi
	@if ! command -v pylint >/dev/null 2>&1; then \
		echo "Note: pylint not found. Install with: sudo apt install pylint"; \
	else \
		echo "Running pylint..."; \
		$(PYTHON) -m pylint ros2systemd --rcfile=.pylintrc 2>/dev/null || \
			$(PYTHON) -m pylint ros2systemd; \
	fi

.PHONY: wheel
wheel:
	@echo "Building Python wheel..."
	@if [ -d "dist" ]; then \
		echo "Cleaning previous dist directory..."; \
		rm -rf dist; \
	fi
	@if [ -d "ros2systemd.egg-info" ]; then \
		echo "Cleaning previous egg-info..."; \
		rm -rf ros2systemd.egg-info; \
	fi
	@$(PYTHON) -m pip install --upgrade setuptools wheel build 2>/dev/null || true
	@$(PYTHON) -m build --wheel --outdir dist .
	@echo "Wheel built successfully:"
	@ls -lh dist/*.whl
	@echo ""
	@echo "To install locally: pip install dist/*.whl"
	@echo "To upload to PyPI: twine upload dist/*.whl"

.PHONY: release
release: clean lint format wheel
	@echo "Release preparation complete!"
	@echo ""
	@echo "Built wheel:"
	@ls -lh dist/*.whl
	@echo ""
	@echo "Next steps:"
	@echo "1. Commit all changes: git add -A && git commit -m 'Release v$$($(PYTHON) setup.py --version)'"
	@echo "2. Tag the release: git tag v$$($(PYTHON) setup.py --version)"
	@echo "3. Push to GitHub: git push origin main --tags"
	@echo "4. Upload wheel to GitHub release page"
	@echo "5. (Optional) Upload to PyPI: twine upload dist/*.whl"
