from ultralytics import YOLO
import wandb
import os
import configparser

class YOLOTrainer:
    def __init__(self, config_file):
        self.config = self.load_config(config_file)
        print(self.config)
        wandb.login(key=self.config['wandb']['key'])
        wandb.init(project='Jajusung_YOLOseg', config=self.config)
        self.model = YOLO(self.config['model']['weights'])

    def load_config(self, config_file):
        config = configparser.ConfigParser()
        config.read(config_file)

        # Convert configparser object to dictionary with proper types
        config_dict = {section: {key: self.convert_value(value) for key, value in config.items(section)} for section in config.sections()}
        return config_dict

    def convert_value(self, value):
        # Convert string values to appropriate types
        try:
            if value.isdigit():
                return int(value)
            elif value.replace('.', '', 1).isdigit():
                return float(value)
            elif value.lower() in ['true', 'false']:
                return value.lower() == 'true'
            return value.strip("'")  # Remove quotes if present
        except ValueError:
            return value

    def train_model(self):
        data_path = "segs/data.yaml"
        if not os.path.exists(data_path):
            raise FileNotFoundError(f"Dataset config file not found: {data_path}")

        self.model.train(data=data_path, **self.config['train_config'])

if __name__ == "__main__":
    trainer = YOLOTrainer("config.conf")
    trainer.train_model()
