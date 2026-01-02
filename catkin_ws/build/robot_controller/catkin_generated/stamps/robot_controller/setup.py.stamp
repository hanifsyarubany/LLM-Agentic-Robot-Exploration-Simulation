from dotenv import load_dotenv
import os, requests, json
import rospy
import rospkg
load_dotenv()
API_KEY = os.getenv("OPENAI_API_KEY")
URL = "https://api.openai.com/v1/responses"

rospack = rospkg.RosPack()
pkg_path = rospack.get_path("semantic_mapping")
semantic_map_path = os.path.join(pkg_path, "maps", "semantic_map.json")

direction_keys = {
    "1": "left",
    "2":"straight",
    "3":"right"
}
