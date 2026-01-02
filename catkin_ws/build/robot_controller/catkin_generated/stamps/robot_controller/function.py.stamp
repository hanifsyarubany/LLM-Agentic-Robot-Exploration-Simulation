from setup import *
from prompt_template import *

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
    
def llm_direction_guide(current_junction, target_stores):
    with open(semantic_map_path, "r") as f:
        semantic_map= json.load(f)
    available_actions = [i.split(" --- ")[0] for i in current_junction["poi_pairs"]]
    try:
        response = chat_completion(system_navigation_prompt,user_navigation_prompt.format(
            target_stores=target_stores,
            current_junction=current_junction,
            available_actions=available_actions,
            semantic_map=semantic_map
        ))
        print(response)
        if len(response)==1 and response.isnumeric():
            return direction_keys[response]
        else:
            return -1

    except:
        return -1