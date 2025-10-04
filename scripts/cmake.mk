# Defaults (can be overridden from CLI: make debug target=myapp)
target ?=
cmake ?=
cxx ?=
cross ?=
compiler ?=
jobs ?= $(shell nproc 2>/dev/null || echo 4)

# Build script path (adjust if using system-wide installation)
BUILD_SCRIPT := cbuild
# If installed system-wide, use: BUILD_SCRIPT := cmk

# Common build flags
BUILD_FLAGS = $(if $(target),-t $(target)) \
              $(if $(compiler),--compiler $(compiler)) \
              $(if $(cmake),--cmake-args $(cmake)) \
              $(if $(cxx),--cxx-flags $(cxx)) \
              $(if $(cross),--cross $(cross)) \
              -j $(jobs)

ifdef cache
	BUILD_FLAGS += --ccache
endif

ifdef verbose
	BUILD_FLAGS += --verbose
endif

ifdef coverage
	BUILD_FLAGS += --coverage
endif

# ============================================================================
# Primary Targets
# ============================================================================

.DEFAULT_GOAL := all

# Quick build targets
.PHONY: all build b
all build b: debug

.PHONY: rebuild rb
rebuild rb: clean debug

# Debug build
.PHONY: debug d
debug d: format
	@$(BUILD_SCRIPT) -b Debug --ccache $(BUILD_FLAGS)

# Release build
.PHONY: release r
release r: format
	@$(BUILD_SCRIPT) -b Release $(BUILD_FLAGS)

# RelWithDebInfo build
.PHONY: relwithdebinfo rwdi
relwithdebinfo rwdi: format
	@$(BUILD_SCRIPT) -b RelWithDebInfo $(BUILD_FLAGS)

# MinSizeRel build
.PHONY: minsizerel msr
minsizerel msr: format
	@$(BUILD_SCRIPT) -b MinSizeRel $(BUILD_FLAGS)

# Benchmark optimized build
.PHONY: benchmark bench
benchmark bench: format
	@$(BUILD_SCRIPT) -b Release --benchmark $(BUILD_FLAGS)

# ============================================================================
# Install Targets
# ============================================================================

.PHONY: install i
install i:
	@$(BUILD_SCRIPT) -b Release --install $(BUILD_FLAGS)

.PHONY: install-debug id
install-debug id:
	@$(BUILD_SCRIPT) -b Debug --install $(BUILD_FLAGS)

# ============================================================================
# Testing Targets
# ============================================================================

.PHONY: test t
test t:
	@$(BUILD_SCRIPT) -b Debug --test $(BUILD_FLAGS)

.PHONY: test-release tr
test-release tr:
	@$(BUILD_SCRIPT) -b Release --test $(BUILD_FLAGS)

.PHONY: test-verbose tv
test-verbose tv:
	@$(BUILD_SCRIPT) -b Debug --test --verbose $(BUILD_FLAGS)

# Coverage with tests
.PHONY: coverage cov
coverage cov: format
	@$(BUILD_SCRIPT) -b Debug --coverage --test $(BUILD_FLAGS)
	@echo ""
	@echo "📊 Generating coverage report..."
	@lcov --capture --directory build --output-file coverage.info 2>/dev/null || \
		{ echo "❌ Coverage capture failed"; exit 1; }
	@lcov --remove coverage.info '/usr/*' '*/test/*' '*/tests/*' --output-file coverage.info
	@genhtml coverage.info --output-directory coverage_report 2>/dev/null || \
		{ echo "❌ Coverage report generation failed"; exit 1; }
	@echo "✅ Coverage report generated in coverage_report/"


# ============================================================================
# Sanitizer Builds
# ============================================================================

.PHONY: asan address
asan address: format
	@$(BUILD_SCRIPT) -b Debug -s asan $(BUILD_FLAGS)


.PHONY: msan memory
msan memory: format
	@$(BUILD_SCRIPT) -b Debug -s msan $(BUILD_FLAGS)


.PHONY: tsan thread
tsan thread: format
	@$(BUILD_SCRIPT) -b Debug -s tsan $(BUILD_FLAGS)

.PHONY: ubsan ub
ubsan ub: format
	@$(BUILD_SCRIPT) -b Debug -s ubsan $(BUILD_FLAGS)


# ============================================================================
# Cross-Compilation Targets
# ============================================================================

.PHONY: cross-arm32
cross-arm32: format
	@$(BUILD_SCRIPT) -b Release --cross arm32 $(BUILD_FLAGS)

.PHONY: cross-aarch64 cross-arm64
cross-aarch64 cross-arm64: format
	@$(BUILD_SCRIPT) -b Release --cross aarch64 $(BUILD_FLAGS)

.PHONY: cross-rpi3
cross-rpi3: format
	@$(BUILD_SCRIPT) -b Release --cross rpi3 $(BUILD_FLAGS)

.PHONY: cross-rpi4
cross-rpi4: format
	@$(BUILD_SCRIPT) -b Release --cross rpi4 $(BUILD_FLAGS)

# List available cross-compilation targets
.PHONY: list-cross
list-cross:
	@$(BUILD_SCRIPT) --list-cross-targets


# ============================================================================
# Maintenance & Utilities
# ============================================================================

.PHONY: clean c
clean c:
	@$(BUILD_SCRIPT) --clean

.PHONY: clean-all ca
clean-all ca:
	@echo "🧹 Cleaning all build artifacts..."
	@rm -rf build/ install/ .cache/ .ccache/
	@rm -f compile_commands.json .build.config
	@rm -f *.cmake clang-tidy.log cppcheck.log
	@rm -rf coverage.info coverage_report/
	@rm -rf docs/html/ docs/latex/
	@echo "✅ Workspace cleaned"

.PHONY: stats info
stats info:
	@$(BUILD_SCRIPT) --stats

.PHONY: list-targets lt
list-targets lt:
	@$(BUILD_SCRIPT) --list-targets

# Dry-run to see commands
.PHONY: dry-run dr
dry-run dr:
	@$(BUILD_SCRIPT) -b Debug --dry-run $(BUILD_FLAGS)

# ============================================================================
# Code Formatting
# ============================================================================

.PHONY: format fmt
format fmt:
	@echo "🎨 Formatting code..."
	@if command -v clang-format >/dev/null 2>&1; then \
		find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" -o -name "*.cxx" \) \
			! -path "./build/*" ! -path "./install/*" ! -path "./third_party/*" \
			-exec clang-format -i -style=file {} + 2>/dev/null && \
		echo "  ✓ C++ files formatted"; \
	else \
		echo "  ⚠️  clang-format not found"; \
	fi
	@if command -v cmake-format >/dev/null 2>&1; then \
		find . -type f -name "CMakeLists.txt" -o -name "*.cmake" \
			! -path "./build/*" ! -path "./install/*" \
			-exec cmake-format -i {} + 2>/dev/null && \
		echo "  ✓ CMake files formatted"; \
	else \
		echo "  ⚠️  cmake-format not found"; \
	fi
	@if command -v black >/dev/null 2>&1; then \
		find . -type f -name "*.py" ! -path "./build/*" ! -path "./install/*" \
			-exec black {} + 2>/dev/null && \
		echo "  ✓ Python files formatted"; \
	else \
		echo "  ⚠️  black not found"; \
	fi
	@echo "✅ Formatting complete"

.PHONY: format-check fc
format-check fc:
	@echo "🔍 Checking code formatting..."
	@if command -v clang-format >/dev/null 2>&1; then \
		find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
			! -path "./build/*" ! -path "./install/*" \
			-exec clang-format -style=file --dry-run --Werror {} + 2>&1 | \
		grep -v "warning:" || echo "  ✓ C++ formatting OK"; \
	fi

# ============================================================================
# Documentation
# ============================================================================

.PHONY: docs doc
docs doc:
	@echo "📚 Generating documentation..."
	@if [ -f Doxyfile ]; then \
		doxygen Doxyfile 2>/dev/null && \
		echo "✅ Documentation generated in docs/html/"; \
	elif [ -f docs/Doxyfile ]; then \
		cd docs && doxygen Doxyfile 2>/dev/null && \
		echo "✅ Documentation generated in docs/html/"; \
	else \
		echo "❌ Doxyfile not found"; \
		exit 1; \
	fi

.PHONY: view-docs vd
view-docs vd:
	@xdg-open docs/html/index.html 2>/dev/null || open docs/html/index.html || \
		{ echo "❌ Failed to open documentation"; exit 1; }

# ============================================================================
# Development Tools
# ============================================================================

.PHONY: configure conf
configure conf:
	@echo "⚙️  Configuring project..."
	@cmake -B build -G Ninja $(if $(cmake),$(cmake))

.PHONY: reconfigure reconf
reconfigure reconf: clean configure

.PHONY: ccache-stats cs
ccache-stats cs:
	@if command -v ccache >/dev/null 2>&1; then \
		echo "📊 ccache statistics:"; \
		ccache -s; \
	else \
		echo "❌ ccache not installed"; \
	fi

.PHONY: ccache-clear cc
ccache-clear cc:
	@if command -v ccache >/dev/null 2>&1; then \
		ccache -C; \
		echo "✅ ccache cleared"; \
	else \
		echo "❌ ccache not installed"; \
	fi

# ============================================================================
# IDE Support
# ============================================================================

.PHONY: compile-commands compile-db
compile-commands compile-db:
	@$(BUILD_SCRIPT) -b Debug
	@if [ -f build/compile_commands.json ]; then \
		ln -sf build/compile_commands.json . && \
		echo "✅ compile_commands.json symlink created"; \
	else \
		echo "❌ compile_commands.json not found"; \
	fi

.PHONY: clangd
clangd:
	@echo "⚙️  Setting up clangd..."
	@echo "CompileFlags:" > .clangd
	@echo "  Add: [-Wall, -Wextra, -std=c++17]" >> .clangd
	@echo "  Remove: [-march=*, -mtune=*]" >> .clangd
	@echo "---" >> .clangd
	@echo "Diagnostics:" >> .clangd
	@echo "  UnusedIncludes: Strict" >> .clangd
	@echo "  MissingIncludes: Strict" >> .clangd
	@make compile-commands
	@echo "✅ clangd configured"

# ============================================================================
# Docker Targets
# ============================================================================

.PHONY: docker-build db
docker-build db:
	@if [ -f Dockerfile ]; then \
		echo "🐳 Building Docker image..."; \
		docker build -t $(shell basename $(CURDIR)):latest .; \
	else \
		echo "❌ Dockerfile not found"; \
		exit 1; \
	fi

.PHONY: docker-run drun
docker-run drun:
	@docker run -it --rm \
		-v $(PWD):/workspace \
		-w /workspace \
		$(shell basename $(CURDIR)):latest

.PHONY: docker-shell dsh
docker-shell dsh:
	@docker run -it --rm \
		-v $(PWD):/workspace \
		-w /workspace \
		$(shell basename $(CURDIR)):latest /bin/bash

# ============================================================================
# Continuous Integration
# ============================================================================

.PHONY: ci
ci: format-check lint test

.PHONY: ci-full
ci-full: clean format-check lint coverage

# ============================================================================
# Package Management
# ============================================================================

.PHONY: deps dep
deps dep:
	@echo "📦 Installing dependencies..."
	@if [ -f conan.txt ] || [ -f conanfile.txt ]; then \
		conan install . --build=missing && \
		echo "✅ Conan dependencies installed"; \
	elif [ -f vcpkg.json ]; then \
		vcpkg install && \
		echo "✅ vcpkg dependencies installed"; \
	elif [ -f requirements.txt ]; then \
		pip install -r requirements.txt && \
		echo "✅ Python dependencies installed"; \
	else \
		echo "⚠️  No dependency file found (conan.txt, vcpkg.json, requirements.txt)"; \
	fi

# ============================================================================
# Benchmarking
# ============================================================================

.PHONY: bench-build bb
bench-build bb: benchmark

.PHONY: bench-run br
bench-run br: benchmark
	@echo "🏃 Running benchmarks..."
	@if [ -n "$(target)" ]; then \
		./build/$(target); \
	else \
		find build -type f -executable -name "*bench*" -exec {} \;; \
	fi

# ============================================================================
# Static Analysis
# ============================================================================

.PHONY: analyze
analyze:
	@echo "🔍 Running static analysis..."
	@if command -v cppcheck >/dev/null 2>&1; then \
		cppcheck --enable=all --suppress=missingIncludeSystem \
			--project=build/compile_commands.json \
			--xml --xml-version=2 2> cppcheck_report.xml && \
		echo "✅ cppcheck report: cppcheck_report.xml"; \
	fi
	@if command -v clang-tidy >/dev/null 2>&1; then \
		$(BUILD_SCRIPT) --lint; \
	fi

.PHONY: valgrind memcheck
valgrind memcheck:
	@echo "🔍 Running Valgrind memory check..."
	@if [ -n "$(target)" ]; then \
		valgrind --leak-check=full --show-leak-kinds=all \
			--track-origins=yes --verbose \
			./build/$(target); \
	else \
		echo "❌ Specify target: make valgrind target=myapp"; \
		exit 1; \
	fi

# ============================================================================
# Help Target
# ============================================================================

.PHONY: help h
help h:
	@echo "════════════════════════════════════════════════════════════════════"
	@echo "                    CMake Project Makefile"
	@echo "════════════════════════════════════════════════════════════════════"
	@echo ""
	@echo "📦 Quick Build Targets:"
	@echo "  make [d|debug]           Debug build with formatting"
	@echo "  make [r|release]         Release build with formatting"
	@echo "  make [bench|benchmark]   Release with LTO optimizations"
	@echo "  make rwdi                RelWithDebInfo build"
	@echo "  make msr                 MinSizeRel build"
	@echo "  make [rb|rebuild]        Clean and rebuild"
	@echo "  make [i|install]         Build and install (Release)"
	@echo "  make id                  Build and install (Debug)"
	@echo ""
	@echo "🧪 Testing & Coverage:"
	@echo "  make [t|test]            Run tests (Debug)"
	@echo "  make tr                  Run tests (Release)"
	@echo "  make tv                  Run tests (verbose)"
	@echo "  make [cov|coverage]      Generate coverage report"
	@echo "  make vc                  View coverage report"
	@echo ""
	@echo "🔧 Sanitizer Builds:"
	@echo "  make [asan|address]      AddressSanitizer build"
	@echo "  make at                  ASan build + tests"
	@echo "  make [msan|memory]       MemorySanitizer build"
	@echo "  make mt                  MSan build + tests"
	@echo "  make [tsan|thread]       ThreadSanitizer build"
	@echo "  make tt                  TSan build + tests"
	@echo "  make [ubsan|ub]          UndefinedBehaviorSanitizer"
	@echo "  make ut                  UBSan build + tests"
	@echo ""
	@echo "🌍 Cross-Compilation:"
	@echo "  make cross-arm32         Build for ARM 32-bit"
	@echo "  make cross-aarch64       Build for ARM 64-bit"
	@echo "  make cross-rpi3          Build for Raspberry Pi 3"
	@echo "  make cross-rpi4          Build for Raspberry Pi 4"
	@echo "  make list-cross          List available targets"
	@echo ""
	@echo "🔍 Code Quality:"
	@echo "  make [l|lint]            Run linters (clang-tidy, cppcheck)"
	@echo "  make lf                  Format + lint"
	@echo "  make check               Format + lint + test"
	@echo "  make check-all           Format + lint + coverage"
	@echo "  make analyze             Run static analysis"
	@echo "  make valgrind target=X   Run Valgrind on target"
	@echo ""
	@echo "🧹 Maintenance:"
	@echo "  make [c|clean]           Clean build artifacts"
	@echo "  make clean-all           Clean everything"
	@echo "  make [dep|deps]          Install dependencies"
	@echo "  make [info|stats]        Show build statistics"
	@echo "  make [lt|list-targets]   List build targets"
	@echo "  make [dr|dry-run]        Show build commands"
	@echo ""
	@echo "🎨 Code Formatting:"
	@echo "  make [fmt|format]        Format all code"
	@echo "  make fc                  Check formatting"
	@echo ""
	@echo "📚 Documentation:"
	@echo "  make [doc|docs]          Generate documentation"
	@echo "  make [vd|view-docs]      View documentation"
	@echo ""
	@echo "💻 IDE Support:"
	@echo "  make compile-db          Generate compile_commands.json"
	@echo "  make clangd              Setup clangd configuration"
	@echo "  make [cs|ccache-stats]   Show ccache statistics"
	@echo "  make [cc|ccache-clear]   Clear ccache"
	@echo ""
	@echo "🐳 Docker:"
	@echo "  make [db|docker-build]   Build Docker image"
	@echo "  make drun                Run in Docker container"
	@echo "  make dsh                 Open Docker shell"
	@echo ""
	@echo "⚡ Benchmarking:"
	@echo "  make bb                  Build benchmarks"
	@echo "  make br [target=X]       Run benchmarks"
	@echo ""
	@echo "🔄 CI/CD:"
	@echo "  make ci                  Quick CI (format-check + lint + test)"
	@echo "  make ci-full             Full CI (+ coverage)"
	@echo ""
	@echo "💡 Variables (override from command line):"
	@echo "  target=TARGET            Build specific target(s)"
	@echo "  compiler=gcc|clang       Select compiler"
	@echo "  cross=TARGET             Cross-compilation target"
	@echo "  cmake='FLAGS'            Additional CMake flags"
	@echo "  cxx='FLAGS'              Additional C++ flags"
	@echo "  jobs=N                   Parallel jobs (default: $(jobs))"
	@echo "  cache=1                  Enable ccache"
	@echo "  verbose=1                Enable verbose output"
	@echo "  coverage=1               Enable coverage"
	@echo ""
	@echo "📋 Examples:"
	@echo "  make debug target=myapp jobs=8"
	@echo "  make release target=mylib compiler=clang"
	@echo "  make cross-aarch64 target=myapp"
	@echo "  make asan target=myapp cache=1"
	@echo "  make test verbose=1"
	@echo "  make coverage"
	@echo "  make benchmark target=bench_suite"
	@echo ""
	@echo "════════════════════════════════════════════════════════════════════"

.PHONY: version v
version v:
	@echo "CMake Build System v2.0"
	@echo "Features: Cross-compilation, Docker, Sanitizers, Coverage, Lint"
	@$(BUILD_SCRIPT) --help | head -1 || echo "Build script: $(BUILD_SCRIPT)"
