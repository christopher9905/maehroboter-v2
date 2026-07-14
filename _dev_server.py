"""Dev server for previewing the web interface."""
import uvicorn
from mower.executive.mission_executive import MissionExecutive
from mower.api.app import create_app

executive = MissionExecutive(hardware_interface=None)
app = create_app(executive)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
