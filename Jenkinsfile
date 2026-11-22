// MultiRobotCore CI pipeline (Jenkins only; no GitHub Actions).
//
// Designed for a single CPU-only Linux agent with ~16 GB RAM: one build at a
// time, bounded compile parallelism, sanitizer and clang-tidy trees removed
// after use. Animations are not re-rendered on normal builds.
//
// Agent requirements: CMake >= 3.24, Ninja, GCC >= 12 or Clang >= 15,
// clang-format, clang-tidy, Python >= 3.11 with venv, git. Network access is
// needed only for the first dependency download (archives are cached in
// .deps/downloads, which the workspace keeps between builds).

pipeline {
  agent { label 'linux' }

  options {
    disableConcurrentBuilds()
    timestamps()
    timeout(time: 3, unit: 'HOURS')
    buildDiscarder(logRotator(numToKeepStr: '20'))
    skipDefaultCheckout()
  }

  parameters {
    string(name: 'BUILD_JOBS', defaultValue: '4', description: 'Parallel compile/test jobs (keep <= 4 on a 16 GB machine)')
    booleanParam(name: 'RUN_SANITIZERS', defaultValue: true, description: 'ASan + UBSan build and tests')
    booleanParam(name: 'RUN_CLANG_TIDY', defaultValue: true, description: 'Static analysis of library sources')
    booleanParam(name: 'RUN_BENCHMARKS', defaultValue: false, description: 'Run experiment sweeps and regenerate benchmark pages')
    booleanParam(name: 'RENDER_DOCS', defaultValue: false, description: 'Re-render all README/documentation animations (slow)')
    booleanParam(name: 'BUILD_ROS2', defaultValue: false, description: 'Build and test the optional ROS 2 adapter (needs a ROS 2 environment or Docker)')
  }

  environment {
    CMAKE_BUILD_PARALLEL_LEVEL = "${params.BUILD_JOBS}"
    CTEST_PARALLEL_LEVEL = "${params.BUILD_JOBS}"
    CTEST_OUTPUT_ON_FAILURE = '1'
    PYTHON = "${env.WORKSPACE}/.venv/bin/python"
  }

  stages {
    stage('Checkout') {
      steps {
        checkout scm
        sh 'git log -1 --format="%h %an %s"'
      }
    }

    stage('Environment Validation') {
      steps {
        sh 'scripts/setup/check_environment.sh'
        sh 'test -x .venv/bin/python || scripts/setup/bootstrap_python.sh python3'
      }
    }

    stage('CMake Configure') {
      steps {
        sh 'cmake --preset ci-debug'
      }
    }

    stage('Debug Build') {
      steps {
        sh 'cmake --build --preset ci-debug'
      }
    }

    stage('Unit Tests') {
      steps {
        sh 'ctest --test-dir build/ci-debug -L unit --output-junit junit-unit.xml'
      }
    }

    stage('Numerical Tests') {
      steps {
        sh 'ctest --test-dir build/ci-debug -L numerical --output-junit junit-numerical.xml'
      }
    }

    stage('Integration Tests') {
      steps {
        sh 'ctest --test-dir build/ci-debug -L integration --output-junit junit-integration.xml'
      }
    }

    stage('MAPF Regression Tests') {
      steps {
        sh 'ctest --test-dir build/ci-debug -L regression --output-junit junit-regression.xml'
      }
    }

    stage('Release Build') {
      steps {
        sh 'cmake --preset ci-release'
        sh 'cmake --build --preset ci-release'
        sh 'ctest --test-dir build/ci-release --output-junit junit-release.xml'
      }
    }

    stage('clang-format Check') {
      steps {
        sh 'scripts/ci/check_format.sh'
      }
    }

    stage('clang-tidy') {
      when { expression { params.RUN_CLANG_TIDY } }
      steps {
        sh "scripts/ci/run_clang_tidy.sh build/ci-debug clang-tidy ${params.BUILD_JOBS}"
      }
    }

    stage('Python Ruff') {
      steps {
        sh '$PYTHON -m ruff check python scripts ros2'
        sh '$PYTHON -m ruff format --check python scripts ros2'
      }
    }

    stage('Python Tests') {
      steps {
        sh '$PYTHON -m pytest --junitxml=build/junit-python.xml'
      }
    }

    stage('Sanitizer Build') {
      when { expression { params.RUN_SANITIZERS } }
      steps {
        sh 'cmake --preset sanitize'
        sh 'cmake --build --preset sanitize'
        sh 'ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 ctest --test-dir build/sanitize --output-junit junit-sanitize.xml'
      }
      post {
        always { sh 'rm -rf build/sanitize' }
      }
    }

    stage('Demo Smoke Tests') {
      steps {
        sh 'scripts/ci/smoke_demos.sh build/ci-release'
      }
    }

    stage('Install/Package Test') {
      steps {
        sh 'scripts/ci/install_test.sh build/ci-release Release'
      }
    }

    stage('ROS 2 Adapter') {
      when { expression { params.BUILD_ROS2 } }
      steps {
        sh 'scripts/ci/ros2_build.sh'
      }
    }

    stage('Benchmarks') {
      when { expression { params.RUN_BENCHMARKS } }
      steps {
        sh '$PYTHON scripts/benchmark/run_all.py --build-dir build/ci-release'
      }
    }

    stage('Render Documentation Assets') {
      when { expression { params.RENDER_DOCS } }
      steps {
        sh '$PYTHON scripts/render/render_all.py --build-dir build/ci-release'
        archiveArtifacts artifacts: 'docs/assets/**/*.gif, docs/assets/manifest.json', allowEmptyArchive: true
      }
    }
  }

  post {
    always {
      junit testResults: 'build/**/junit-*.xml', allowEmptyResults: true
    }
    cleanup {
      sh 'rm -rf results'
    }
  }
}
