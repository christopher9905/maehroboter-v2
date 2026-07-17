"""Start the application with simulated devices instead of physical ones."""

from __future__ import annotations

import os

import uvicorn


def main() -> None:
    os.environ["MV2_MODE"] = "simulation"
    host = os.environ.get("MV2_SIM_HOST", "127.0.0.1")
    port = int(os.environ.get("MV2_SIM_PORT", "8090"))
    uvicorn.run("mower.main:app", host=host, port=port)


if __name__ == "__main__":
    main()
