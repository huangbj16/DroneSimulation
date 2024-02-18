import json

settings_filename = "study_settings.json"

class StudyConfiguration:
    def __init__(self) -> None:
        with open(settings_filename, 'r') as f:
            settings_data = json.load(f)
            print(settings_data)
        pass

if __name__ == "__main__":
    config = StudyConfiguration()
    