#!/usr/bin/python3

import json

def write_promps(filename, data):
	with open(filename, 'w', encoding='utf-8') as f:
		json.dump(data, f, ensure_ascii=False, indent=4)

