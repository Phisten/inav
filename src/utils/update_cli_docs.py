#!/usr/bin/env python3

import yaml  # pyyaml / python-yaml

SETTINGS_MD_PATH = "docs/Settings.md"
SETTINGS_YAML_PATH = "src/main/fc/settings.yaml"

def parse_settings_yaml():
    """Parse the YAML settings specs"""

    with open(SETTINGS_YAML_PATH, "r") as settings_yaml:
        return yaml.load(settings_yaml, Loader=yaml.Loader)

def generate_md_table_from_yaml(settings_yaml):
    """Generate a sorted markdown table with description & default value for each setting"""
    params = {}
    
    # Extract description and default value of each setting from the YAML specs (if present)
    for group in settings_yaml['groups']:
        for member in group['members']:
            if any(key in member for key in ["description", "default_value"]):
                params[member['name']] = {
                        "description": member["description"] if "description" in member else "",
                        "default": member["default_value"] if "default_value" in member else ""
                    }
    
    # MD table header
    md_table_lines = [
        "| Variable Name | Default Value | Description |\n",
        "| ------------- | ------------- | ----------- |\n",
        ]
    
    # Sort the settings by name and build the rows of the table
    for param in sorted(params.items()):
        md_table_lines.append("| {} | {} | {} |\n".format(param[0], param[1]['default'], param[1]['description']))
    
    # Return the assembled table
    return md_table_lines

def write_settings_md(lines):
    """Write the contents of the CLI settings docs"""

    with open(SETTINGS_MD_PATH, "w") as settings_md:
        settings_md.writelines(lines)

if __name__ == "__main__":
    settings_yaml = parse_settings_yaml()
    md_table_lines = generate_md_table_from_yaml(settings_yaml)
    settings_md_lines = \
        ["# CLI Variable Reference\n", "\n" ] + \
        md_table_lines + \
        ["\n", "> Note: this table is autogenerated. Do not edit it manually."]
    write_settings_md(settings_md_lines)