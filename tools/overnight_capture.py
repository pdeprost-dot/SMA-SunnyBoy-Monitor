"""Bounded, read-only MQTT/API capture for natural SMA transition studies."""

import argparse
import json
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

import paho.mqtt.client as mqtt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--broker", required=True)
    parser.add_argument("--esp", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--max-bytes", type=int, default=4 * 1024 * 1024)
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    def append(kind: str, data: object) -> None:
        if output.exists() and output.stat().st_size >= args.max_bytes:
            return
        record = {
            "received_at": datetime.now(timezone.utc).isoformat(),
            "kind": kind,
            "data": data,
        }
        with output.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, separators=(",", ":")) + "\n")

    def on_connect(client, _userdata, _flags, reason_code, _properties):
        append("mqtt_connection", {"connected": int(reason_code) == 0, "reason": int(reason_code)})
        if int(reason_code) == 0:
            client.subscribe("smaesp/inv+", qos=0)

    def on_disconnect(_client, _userdata, _flags, reason_code, _properties):
        append("mqtt_connection", {"connected": False, "reason": int(reason_code)})

    def on_message(_client, _userdata, message):
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except Exception:
            payload = {"decode_error": True, "bytes": len(message.payload)}
        append("mqtt_snapshot", {"topic": message.topic, "retained": message.retain, "payload": payload})

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="sma-overnight-observer")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.connect_async(args.broker, 1883, 30)
    client.loop_start()

    next_poll = 0.0
    while True:
        now = time.monotonic()
        if now >= next_poll:
            compact = {}
            for name in ("status", "scheduler/status"):
                try:
                    with urllib.request.urlopen(f"http://{args.esp}/api/{name}", timeout=5) as response:
                        value = json.load(response)
                    if name == "status":
                        compact["system"] = {
                            "uptimeMs": value.get("uptimeMs"),
                            "reset": value.get("system"),
                            "heap": value.get("heap"),
                            "wifi": value.get("wifi", {}).get("connected"),
                            "mqtt": value.get("mqtt", {}).get("mqttConnected"),
                            "ota": value.get("ota"),
                        }
                    else:
                        compact["scheduler"] = value
                except Exception as exc:
                    compact[name] = {"error": type(exc).__name__}
            append("system_sample", compact)
            next_poll = now + 60.0
        time.sleep(1.0)


if __name__ == "__main__":
    main()
