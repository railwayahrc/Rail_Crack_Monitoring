from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import subprocess

app = FastAPI()

# Allow all origins for testing. Restrict as needed for production.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],         # Allow all origins
    allow_credentials=True,
    allow_methods=["*"],         # Allow all HTTP methods
    allow_headers=["*"],         # Allow all headers
)

@app.post("/set_param")
async def set_param(request: Request):
    try:
        data = await request.json()
        print("Raw JSON received:", data)
    except Exception as e:
        body = await request.body()
        print("Failed to parse JSON. Raw body:", body)
        return JSONResponse(
            status_code=400,
            content={"error": "Invalid JSON", "detail": str(e), "body": body.decode(errors='replace')}
        )

    # Validate required keys
    required_keys = {"cam_index", "param", "value"}
    if not required_keys.issubset(data):
        missing = required_keys - set(data.keys())
        print("Missing keys in JSON:", missing)
        return JSONResponse(
            status_code=400,
            content={"error": "Missing required keys", "missing": list(missing), "received": list(data.keys())}
        )

    param_full = f"camera_{data['cam_index']}.{data['param']}"
    cmd = [
        "ros2", "param", "set",
        "/multi_camera_parallel_node",
        param_full, str(data['value'])
    ]
    try:
        output = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
        return JSONResponse(
            status_code=200,
            content={"success": True, "output": output.decode()}
        )
    except subprocess.CalledProcessError as e:
        return JSONResponse(
            status_code=500,
            content={"success": False, "error": e.output.decode()}
        )

