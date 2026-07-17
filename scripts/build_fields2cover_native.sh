#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NATIVE_ROOT="${ROOT}/.native"
SOURCE_DIR="${NATIVE_ROOT}/src/Fields2Cover"
BUILD_DIR="${NATIVE_ROOT}/build/fields2cover"
INSTALL_DIR="${NATIVE_ROOT}/fields2cover"
GDAL_SOURCE_DIR="${NATIVE_ROOT}/src/gdal"
GDAL_BUILD_DIR="${NATIVE_ROOT}/build/gdal"
GDAL_INSTALL_DIR="${NATIVE_ROOT}/gdal"
F2C_COMMIT="92b9f2fcf72d26c44202f9d2ab54fbf25ed87426"
GDAL_TAG="v3.9.3"
PYTHON_BIN="${PYTHON_BIN:-$(command -v python3)}"

mkdir -p "${NATIVE_ROOT}/src" "${BUILD_DIR}" "${INSTALL_DIR}"

if [[ ! -f "${GDAL_INSTALL_DIR}/lib/libgdal.dylib" ]] || \
   ! otool -L "${GDAL_INSTALL_DIR}/lib/libgdal.dylib" 2>/dev/null | grep -q "libgeos"; then
  if [[ ! -d "${GDAL_SOURCE_DIR}/.git" ]]; then
    git clone --filter=blob:none --depth 1 --branch "${GDAL_TAG}" \
      https://github.com/OSGeo/gdal.git "${GDAL_SOURCE_DIR}"
  fi
  cmake -S "${GDAL_SOURCE_DIR}" -B "${GDAL_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_INSTALL_PREFIX="${GDAL_INSTALL_DIR}" \
    -DBUILD_APPS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_TESTING=OFF \
    -DGDAL_BUILD_OPTIONAL_DRIVERS=OFF \
    -DOGR_BUILD_OPTIONAL_DRIVERS=OFF \
    -DGDAL_USE_EXTERNAL_LIBS=OFF \
    -DGDAL_USE_ZLIB=ON \
    -DGDAL_USE_ZLIB_INTERNAL=OFF \
    -DGDAL_USE_PNG=ON \
    -DGDAL_USE_PNG_INTERNAL=OFF \
    -DGDAL_USE_GEOS=ON
  cmake --build "${GDAL_BUILD_DIR}" --parallel "${F2C_BUILD_JOBS:-4}"
  cmake --install "${GDAL_BUILD_DIR}"
fi

if [[ ! -d "${SOURCE_DIR}/.git" ]]; then
  git clone --filter=blob:none --no-checkout \
    https://github.com/Fields2Cover/Fields2Cover.git "${SOURCE_DIR}"
fi

git -C "${SOURCE_DIR}" fetch --depth 1 origin "${F2C_COMMIT}"
git -C "${SOURCE_DIR}" checkout --detach "${F2C_COMMIT}"

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
  -DCMAKE_PREFIX_PATH="${GDAL_INSTALL_DIR};/opt/homebrew" \
  -DGDAL_ROOT="${GDAL_INSTALL_DIR}" \
  -DPython_EXECUTABLE="${PYTHON_BIN}" \
  -DBUILD_PYTHON=ON \
  -DBUILD_TUTORIALS=OFF \
  -DBUILD_DOC=OFF \
  -DBUILD_TESTING=OFF \
  -DALLOW_PARALLELIZATION=ON \
  -DUSE_ORTOOLS_FETCH_SRC=OFF

cmake --build "${BUILD_DIR}" --parallel "${F2C_BUILD_JOBS:-4}"
cmake --install "${BUILD_DIR}"
# Fields2Cover 2.1.0's generated install(CODE) does not quote setup.py or the
# prefix.  Re-run the already built Python package explicitly so workspace
# paths containing spaces are handled correctly.
"${PYTHON_BIN}" "${BUILD_DIR}/swig/python/setup.py" install \
  --prefix="${INSTALL_DIR}"

PYTHON_SITE="$(${PYTHON_BIN} - <<'PY'
import sysconfig
print(sysconfig.get_path("purelib", vars={"base": "", "platbase": ""}).lstrip("/"))
PY
)"

echo "Fields2Cover wurde nach ${INSTALL_DIR} installiert."
echo "Python-Pfad: ${INSTALL_DIR}/${PYTHON_SITE}"
echo "Zum Testen: PYTHONPATH=${INSTALL_DIR}/${PYTHON_SITE} ${PYTHON_BIN} -c 'import fields2cover'"
