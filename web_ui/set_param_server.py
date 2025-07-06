from fastapi import FastAPI
from pydantic import BaseModel
import subprocess
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# Allow requests from your UI server (adjust port if needed)
origins = [
    "http://localhost:8000",  # UI served by python3 -m http.server 8000
    "http://127.0.0.1:8000"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,      # or ["*"] for all origins (less secure)
    allow_credentials=True,
    allow_methods=["*"],        # allow all HTTP methods
    allow_headers=["*"],        # allow all headers
)


class ParamRequest(BaseModel):
    cam_index: int
    param: str
    value: str

@app.post("/set_param")
def set_param(req: ParamRequest):
    param_full = f"camera_{req.cam_index}.{req.param}"
    cmd = [
        "ros2", "param", "set",
        "/multi_camera_parallel_node",
        param_full, req.value
    ]
    try:
        output = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
        return {"success": True, "output": output.decode()}
    except subprocess.CalledProcessError as e:
        return {"success": False, "error": e.output.decode()}
