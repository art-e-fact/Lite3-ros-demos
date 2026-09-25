#!/bin/sh

# ros-jazzy-launch-testing (pulled in by Nav2) ships a pytest plugin that pytest 9
# refuses to load; none of the tests here use it.
if [ -n "${PYTEST_ADDOPTS}" ]; then
    export PYTEST_ADDOPTS="${PYTEST_ADDOPTS} -p no:launch_testing"
else
    export PYTEST_ADDOPTS="-p no:launch_testing"
fi

# Extend PYTEST_ADDOPTS with the JUnit XML report switch only if ARTEFACTS_SCENARIO_UPLOAD_DIR is defined and non-empty.
if [ -n "${ARTEFACTS_SCENARIO_UPLOAD_DIR}" ]; then
    if [ -n "${PYTEST_ADDOPTS}" ]; then
        export PYTEST_ADDOPTS="${PYTEST_ADDOPTS} --junit-xml=${ARTEFACTS_SCENARIO_UPLOAD_DIR}/tests_junit.xml"
    else
        export PYTEST_ADDOPTS="--junit-xml=${ARTEFACTS_SCENARIO_UPLOAD_DIR}/tests_junit.xml"
    fi
fi
