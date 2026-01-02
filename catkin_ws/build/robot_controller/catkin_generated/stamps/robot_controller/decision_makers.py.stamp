from std_msgs.msg import String, Int32
from dotenv import load_dotenv
import os, requests, json, re
import hashlib
import rospy
import rospkg
import threading
import tempfile

load_dotenv()
API_KEY = os.getenv("OPENAI_API_KEY")
URL = "https://api.openai.com/v1/responses"

rospack = rospkg.RosPack()
pkg_path = rospack.get_path("semantic_mapping")
semantic_map_path = os.path.join(pkg_path, "maps", "semantic_map.json")
historical_robot_path = os.path.join(pkg_path, "maps", "historical_robot_trajectory.json")
order_list_path = os.path.join(pkg_path, "maps", "target_object_grasping.json")

ORDER_SCHEMA = {
    "type": "object",
    "properties": {
        "hamburger":   {"type": "integer", "minimum": 0, "maximum": 20},
        "iced_coffee": {"type": "integer", "minimum": 0, "maximum": 20},
        "hot_coffee":  {"type": "integer", "minimum": 0, "maximum": 20},
        "medicine":    {"type": "integer", "minimum": 0, "maximum": 20},
    },
    "required": ["hamburger", "iced_coffee", "hot_coffee", "medicine"],
    "additionalProperties": False
}

direction_keys = {
    "1": "left",
    "2":"straight",
    "3":"right"
}

action_keys = {
    "1":"convenience_store",
    "2":"hamburger_store",
    "3":"pharmacy",
    "4":"cafe",
    "5":"continue"
}

system_navigation_prompt = """
You are a deterministic navigation policy for a robot on a 3-way junction graph.

You must output EXACTLY:
<DIRECTION>|||<STORE_ACTION>

Where:
- DIRECTION: 1=left, 2=straight, 3=right
- STORE_ACTION: 1=convenience_store, 2=hamburger_store, 3=pharmacy, 4=cafe, 5=continue(search)

STRICT OUTPUT RULE:
- Output ONLY digits and the delimiter "|||", nothing else.

------------------------------------------------------------
CRITICAL SEMANTICS (match the real data)
------------------------------------------------------------
- DIRECTION selects which way the robot moves NOW (left/straight/right).
- poi_pairs are NOT mandatory stops; they are POIs visible/accessible in that direction.
- STORE_ACTION indicates whether the robot will STOP and ENTER a POI NOW:
  - If you want to enter a store, output its store action (1-4).
  - Otherwise output 5 (continue), even if a store exists in that direction.
- "pickup" is a POI node (not a store). Entering/going to pickup MUST output STORE_ACTION=5.

------------------------------------------------------------
DATA FORMAT YOU WILL SEE
------------------------------------------------------------
SEMANTIC_MAP is a LIST of junction objects like:
- name: "junction_4"
- poi_pairs: ["left --- pickup", "right --- hamburger_store", ...]
- robot_pose_2d: {x,y,z,yaw}

CURRENT_JUNCTION is one junction object from that same format.

HISTORICAL_ROBOT_DECISION is a TEXT LOG (not structured JSON).

------------------------------------------------------------
POI NAME -> STORE_ACTION MAPPING
------------------------------------------------------------
"convenience_store"  -> 1
"hamburger_store"    -> 2
"pharmacy"           -> 3
"cafe"               -> 4
"pickup"             -> (not a store; never output 1-4 for pickup)

Parsing poi_pairs:
- Each entry is formatted: "<dir> --- <poi_name>"
- <dir> is one of: left, straight, right

------------------------------------------------------------
ITEMS
------------------------------------------------------------
Valid order items:
hamburger, iced_coffee, hot_coffee, medicine

------------------------------------------------------------
STORE CAPABILITIES
------------------------------------------------------------
1 (convenience_store): hamburger, iced_coffee, hot_coffee, medicine (small/mixed)
2 (hamburger_store):   hamburger (primary), iced_coffee/hot_coffee (possible)
3 (pharmacy):          medicine only
4 (cafe):              iced_coffee, hot_coffee

------------------------------------------------------------
STORE INSTANCE IDENTITY (fixes your “same-type store” issue)
------------------------------------------------------------
- A “store” that cannot be visited twice means a STORE INSTANCE at a specific location.
- In this dataset, treat a store instance as uniquely identified by:
    store_instance_id = (junction_name, direction, poi_name)
  (This matches what the robot actually observes at that coordinate/junction.)
- Multiple different store instances can share the same poi_name/store type.
  Visiting one hamburger_store does NOT forbid visiting another hamburger_store elsewhere.
- The “visited” rule applies ONLY to the exact (junction_name, direction, poi_name) triple.

------------------------------------------------------------
HOW TO EXTRACT VISITED STORES FROM THE TEXT LOG
------------------------------------------------------------
From HISTORICAL_ROBOT_DECISION, detect entries like:
- "the robot has stopped at junction X ..."  -> sets current_log_junction = X
- "the robot is turning to the left/straight/right, and then enter the <poi_name>."
  -> add visited_store_instance_id = (current_log_junction, that_direction, poi_name)

Ignore "just continue" (it does NOT mark a store as visited).

------------------------------------------------------------
PICKUP RULE (your new requirement)
------------------------------------------------------------
Let R[item] = max(CURRENT_ORDER_LIST[item], 0)

- If ANY R[item] > 0 (shopping not done):
  - Avoid choosing a DIRECTION whose poi_pairs contains "pickup" in that direction,
    UNLESS no other direction is available.
- If ALL R[item] == 0 (shopping done):
  - The navigation goal becomes reaching pickup:
    - If CURRENT_JUNCTION has a pickup in some direction, choose that direction and output <dir>|||5.
    - Otherwise move in the direction that best heads toward the nearest junction in SEMANTIC_MAP
      that has a pickup in its poi_pairs (geometric heuristic below).

------------------------------------------------------------
TARGET STORE SELECTION (instance-based)
------------------------------------------------------------
Only used when shopping (some R[item] > 0).

Step A) Build the set of candidate store INSTANCES from SEMANTIC_MAP:
- For every junction J in SEMANTIC_MAP:
  for every poi_pair "<dir> --- <poi_name>" where poi_name in {convenience_store, hamburger_store, pharmacy, cafe}:
    candidate instance = (J.name, dir, poi_name)

Step B) Remove visited instances:
- Remove any candidate whose (junction_name, dir, poi_name) is in visited_store_instances.

Step C) Score each remaining candidate instance by how well it matches remaining needs:
Use weights by store type:

Hamburger:
  hamburger_store(type2)=3, convenience_store(type1)=1, else=0
Iced coffee:
  cafe(type4)=3, convenience_store(type1)=2, hamburger_store(type2)=1, pharmacy(type3)=0
Hot coffee:
  cafe(type4)=3, convenience_store(type1)=2, hamburger_store(type2)=1, pharmacy(type3)=0
Medicine:
  pharmacy(type3)=3, convenience_store(type1)=1, else=0

Score(instance) = Σ_item ( R[item] * weight(item, store_type(instance)) )

Choose target_instance = argmax Score(instance).
Tie-breakers (deterministic):
1) choose the candidate whose junction pose is closest (Euclidean) to CURRENT_JUNCTION pose
2) lexicographically smallest (junction_name, dir, poi_name)

If best Score == 0 (no useful store instances known):
- Do NOT enter any store now (STORE_ACTION=5)
- Use exploration movement (below), still obeying pickup avoidance.

------------------------------------------------------------
ENTER vs CONTINUE (when to output STORE_ACTION 1-4)
------------------------------------------------------------
At CURRENT_JUNCTION, parse its poi_pairs into a map: dir -> poi_name (if present).

If the chosen target_instance is located at CURRENT_JUNCTION (same junction_name)
AND its direction dir exists at this junction
AND it is not visited:
- Enter it NOW:
  Output: <dir_number>|||<store_action_number_for_that_poi_name>
Otherwise:
- You are traveling/searching:
  Output STORE_ACTION = 5 always.

------------------------------------------------------------
MOVEMENT DIRECTION HEURISTIC (works with your map format)
------------------------------------------------------------
When you are NOT entering a store now (STORE_ACTION=5):

1) If shopping and there exists a known useful target_instance (best Score > 0):
   - Let target_junction_pose be the pose of the junction that contains target_instance.
   - Compute angle_to_target = atan2(y_t - y_c, x_t - x_c)
   - Compute delta = wrap_to_pi(angle_to_target - yaw_current)
   - Choose direction by delta:
       if |delta| <= π/6  -> straight (2)
       if delta  >  π/6   -> left (1)
       if delta  < -π/6   -> right (3)

2) If shopping but no useful target_instance is known (best Score == 0):
   - Prefer an UNTRIED direction at this junction based on the history log:
     - From the text log, for each "stopped at junction X" followed by "turning to <dir>",
       mark that (X, dir) as tried.
     - Choose among directions not yet tried at CURRENT_JUNCTION.
   - Tie-break direction: straight(2) > left(1) > right(3)

3) Apply pickup avoidance (shopping mode):
   - If shopping (some R[item] > 0) and the selected direction has "pickup" in poi_pairs,
     then choose the best alternative direction (using the same rule) that is NOT pickup.
   - Only take pickup direction if all 3 directions are pickup/forced.

4) If shopping done (all R[item]==0):
   - If pickup exists at CURRENT_JUNCTION, go to it immediately: <pickup_dir>|||5
   - Else choose direction using the same geometric rule toward the nearest junction with pickup.
   - Tie-break direction: straight(2) > left(1) > right(3)

Always output a valid <digit>|||<digit>
"""



user_navigation_prompt = """
----------------------------------------
INPUTS
----------------------------------------

CURRENT_ORDER_LIST:
{order_list}

CURRENT_JUNCTION:
{current_junction}

CURRENT_JUNCTION_AVAILABLE_DIRECTION
{available_actions}

SEMANTIC_MAP:
{semantic_map}

HISTORICAL_ROBOT_DECISION:
{historical_robot_decision}

OUTPUT:
<1|2|3>|||<1|2|3|4|5>
"""

# ---------- Order extraction prompts ----------
system_order_extraction_prompt = """
You are a deterministic order extractor for a delivery robot.

Extract quantities for exactly these items:
- hamburger
- iced_coffee
- hot_coffee
- medicine

Normalization / synonyms:
- hamburger: "hamburger(s)", "burger(s)"
- iced_coffee: "iced coffee", "ice coffee", "cold coffee" (only if explicitly iced/cold)
- hot_coffee: "hot coffee", "coffee" (if not explicitly iced), "americano" (treat as hot coffee)
- medicine: "medicine", "meds", "medication", "emergency medicine",
            "first-aid medication", "first aid", "first-aid"

Quantity rules:
- Parse numerals (e.g., 2) and number words (zero..ten).
- "a" / "an" / "one" / "a pack/box/package" => 1
- If mentioned without a clear quantity ("some"), default to 1
- If mentioned multiple times, sum quantities.
- Optional extras ("if possible", "feel free") STILL count.

Ignore return/delivery phrases ("bring back to depot/loading zone"): they do not change quantities.

Return only the structured JSON required by the schema.
"""

user_order_extraction_prompt = """
USER_INSTRUCTION:
{user_instruction}
"""


def _extract_first_text(result_json: dict) -> str:
    """
    Best-effort extraction of the first text chunk from Responses API output.
    Keeps your code resilient if the output array contains multiple items.
    """
    try:
        for item in result_json.get("output", []):
            for c in item.get("content", []):
                if "text" in c:
                    return c["text"]
    except Exception:
        pass
    return ""

def chat_completion_json_schema(system_prompt: str, user_prompt: str, schema: dict):
    payload = {
        "model": "gpt-4.1-2025-04-14",
        "input": [
            {"role": "system", "content": [{"type": "input_text", "text": system_prompt}]},
            {"role": "user",   "content": [{"type": "input_text", "text": user_prompt}]},
        ],
        "text": {
            "format": {
                "type": "json_schema",
                "name": "order_list",
                "strict": True,
                "schema": schema
            }
        },
        "temperature": 0,
        "max_output_tokens": 128,
        "top_p": 1,
        "store": True,
    }

    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {API_KEY}",
    }

    resp = requests.post(URL, headers=headers, data=json.dumps(payload))
    if resp.status_code != 200:
        rospy.logwarn("LLM ERROR: %s %s", resp.status_code, resp.text)
        return None

    result = resp.json()
    raw = _extract_first_text(result).strip()
    if not raw:
        rospy.logwarn("LLM returned empty output: %s", str(result)[:400])
        return None

    try:
        return json.loads(raw)
    except Exception as e:
        rospy.logwarn("Failed parsing LLM JSON: %s | raw=%s", e, raw)
        return None

def llm_order_list_guide(user_instruction: str) -> dict:
    """
    Returns a JSON dict:
      {"hamburger": int, "iced_coffee": int, "hot_coffee": int, "medicine": int}
    """
    user_prompt = user_order_extraction_prompt.format(user_instruction=user_instruction)
    data = chat_completion_json_schema(system_order_extraction_prompt, user_prompt, ORDER_SCHEMA)

    if not isinstance(data, dict):
        return {"hamburger": 0, "iced_coffee": 0, "hot_coffee": 0, "medicine": 0}

    # Harden types / missing keys:
    out = {
        "hamburger": int(data.get("hamburger", 0) or 0),
        "iced_coffee": int(data.get("iced_coffee", 0) or 0),
        "hot_coffee": int(data.get("hot_coffee", 0) or 0),
        "medicine": int(data.get("medicine", 0) or 0),
    }
    # Clamp to non-negative
    for k in out:
        if out[k] < 0:
            out[k] = 0
    return out

def save_json_atomic(path: str, obj: dict):
    """
    Prevents corrupted JSON if your node is interrupted mid-write.
    """
    d = os.path.dirname(path)
    os.makedirs(d, exist_ok=True)

    fd, tmp_path = tempfile.mkstemp(prefix="._tmp_", suffix=".json", dir=d)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False, indent=2)
        os.replace(tmp_path, path)  # atomic on POSIX
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception:
                pass

def chat_completion(system_prompt, user_prompt):
    payload = {
        "model": "gpt-4.1-2025-04-14",
        "input": [
            {
                "role": "system",
                "content": [
                    {
                        "type": "input_text",
                        "text": system_prompt
                    }
                ]
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "input_text",
                        "text": user_prompt
                    }
                ]
            }
        ],
        "text": {
            "format": {
                "type": "text"
            }
        },
        "reasoning": {},
        "tools": [],
        "temperature": 0,
        "max_output_tokens": 16,
        "top_p": 1,
        "store": True,
    }

    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {API_KEY}",
    }

    resp = requests.post(URL, headers=headers, data=json.dumps(payload))

    # Safety check
    if resp.status_code != 200:
        print("ERROR:", resp.status_code, resp.text)
        return None

    result = resp.json()

    # Extract text output (Responses API)
    try:
        final_text = result["output"][0]["content"][0]["text"]
        return final_text
    except Exception as e:
        print("ERROR extracting text:", e)
        print(result)
        return None
    
def llm_direction_guide(current_junction):
    with open(semantic_map_path, "r") as f:
        semantic_map= json.load(f)
    try:
        with open(historical_robot_path, "r", encoding="utf-8") as f:
            historical_robot = f.read()
        with open(order_list_path, "r") as f:
            order_list = json.load(f)
    except:
        historical_robot = "not available"
    available_actions = [i.split(" --- ")[0] for i in current_junction["poi_pairs"]]
    user_prompt = user_navigation_prompt.format(
        order_list=order_list,
        current_junction=current_junction,
        available_actions=available_actions,
        semantic_map=semantic_map,
        historical_robot_decision=historical_robot
    )
    rospy.loginfo("[DecisionMakersNode] USER PROMPT = %s",user_prompt)
    response = chat_completion(system_navigation_prompt,user_prompt)
    rospy.loginfo("[DecisionMakersNode] LLM response = %s",response)
    return response

class DecisionMakers:
    def __init__(self):
        # -------------------------------------------------------------
        # Parameters
        # -------------------------------------------------------------
        self.semantic_map_topic = rospy.get_param("~semantic_map_topic", "/semantic_mapping_entries")
        self.next_action_topic = rospy.get_param("~next_action_topic", "/next_controller_action")
        self.input_order_list_topic = rospy.get_param("~input_order_list_topic", "/input_order_list")
        # -------------------------------------------------------------
        # Internal State
        # -------------------------------------------------------------
        self._lock = threading.Lock()
        self.order_list = {"hamburger": 0, "iced_coffee": 0, "hot_coffee": 0, "medicine": 0}
        try:
            with open(order_list_path, "r", encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                # keep only expected keys
                for k in self.order_list.keys():
                    self.order_list[k] = int(loaded.get(k, 0) or 0)
        except Exception:
            pass
        # -------------------------------------------------------------
        # Subscribers
        # -------------------------------------------------------------
        self.input_order_list_sub = rospy.Subscriber(
            self.input_order_list_topic,
            String,
            self.input_order_list_callback,
            queue_size=1
        )
        self.semantic_mapping_entries_sub = rospy.Subscriber(
            self.semantic_map_topic,
            String,
            self.semantic_mapping_entries_callback,
            queue_size=1
        )
        # -------------------------------------------------------------
        # Publishers
        # -------------------------------------------------------------
        self.next_action_pub = rospy.Publisher(self.next_action_topic, String, queue_size=1)

        # -------------------------------------------------------------
        # INTERNAT STATE
        # -------------------------------------------------------------
        rospy.loginfo("DecisionMakersNode initialized.")
        rospy.sleep(0.5)

    def input_order_list_callback(self, msg: String):
        user_instruction = (msg.data or "").strip()
        if not user_instruction:
            rospy.logwarn("[DecisionMakersNode] Received empty /input_order_list")
            return

        rospy.loginfo("[DecisionMakersNode] Received user instruction: %s", user_instruction)

        order_dict = llm_order_list_guide(user_instruction)
        rospy.loginfo("[DecisionMakersNode] Parsed order dict: %s", str(order_dict))

        with self._lock:
            self.order_list = order_dict
            try:
                save_json_atomic(order_list_path, self.order_list)
                rospy.loginfo("[DecisionMakersNode] Saved order list to: %s", order_list_path)
            except Exception as e:
                rospy.logwarn("[DecisionMakersNode] Failed saving order list: %s", e)


    def semantic_mapping_entries_callback(self,msg):
        with open(order_list_path, "r") as f:
            current_order = json.load(f)
        try:
            current_junction = json.loads(msg.data)
            rospy.loginfo("[DecisionMakersNode] Current Junction detected = %s",str(current_junction))
        except:
            rospy.loginfo("[DecisionMakersNode] something error in here in receiving current junction")
        available_directions = [i.split(" --- ")[0] for i in current_junction["poi_pairs"]]
        available_pois = [i.split(" --- ")[1] for i in current_junction["poi_pairs"]]
        #  PICKUP PRIORITY
        if [i==0 for i in list(current_order.values())].count(True)==4: 
            if "pickup" in available_pois:
                direction = available_directions[available_pois.index("pickup")]
                next_action = f"{direction}---pickup"
                rospy.loginfo("[DecisionMakersNode] Next Action = %s", next_action)
                msg = String()
                msg.data = next_action
                self.next_action_pub.publish(msg)
                return 
            else:
                poi_pairs = [i for i in current_junction["poi_pairs"]]
                available_directions = [i.split(" --- ")[0] for i in poi_pairs]
                available_pois = [i.split(" --- ")[1] for i in poi_pairs]
                next_action_response =  llm_direction_guide(current_junction)
                try:
                    direction = direction_keys[next_action_response.split("|||")[0]]
                    action = action_keys[next_action_response.split("|||")[1]]
                except:
                    direction = "straight"
                    action = "continue"
                if not direction in available_directions:
                    next_action = f"{available_directions[-1]}---continue"
                else:
                    next_action = f"{direction}---{action}"
                rospy.loginfo("[DecisionMakersNode] Next Action = %s", next_action)
                msg = String()
                msg.data = next_action
                self.next_action_pub.publish(msg)
                return
            
        # NON PICKUP PRIORITY
        poi_pairs = [i for i in current_junction["poi_pairs"] if not "pickup" in i]
        available_directions = [i.split(" --- ")[0] for i in poi_pairs]
        available_pois = [i.split(" --- ")[1] for i in poi_pairs]
        next_action_response =  llm_direction_guide(current_junction)
        try:
            direction = direction_keys[next_action_response.split("|||")[0]]
            action = action_keys[next_action_response.split("|||")[1]]
        except:
            direction = "straight"
            action = "continue"
        if not direction in available_directions:
            next_action = f"{available_directions[-1]}---continue"
        else:
            next_action = f"{direction}---{action}"
        rospy.loginfo("[DecisionMakersNode] Next Action = %s", next_action)
        msg = String()
        msg.data = next_action
        self.next_action_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("decision_makers_node")
    node = DecisionMakers()
    rospy.spin()
