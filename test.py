import os
import re

def update_imports(directory, old_package, new_package):
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".py"):
                file_path = os.path.join(root, file)
                with open(file_path, 'r') as f:
                    content = f.read()
                updated_content = re.sub(rf'\b{old_package}\b', new_package, content)
                with open(file_path, 'w') as f:
                    f.write(updated_content)
                print(f"Updated imports in {file_path}")

if __name__ == "__main__":
    project_directory = "."
    old_package_name = "boardCal"
    new_package_name = "boardCal"
    update_imports(project_directory, old_package_name, new_package_name)