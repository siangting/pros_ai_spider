import csv
from datetime import datetime
import os

class DataCollector:
    def __init__(self):
        self.data = []
        self.path = self._create_directory()

    def _create_directory(self):
        # 用時間創建文件名稱
        timestamp = datetime.now().strftime("%Y%m%d")
        current_dir = os.getcwd()
        path = os.path.join(current_dir, f'training_data_{timestamp}')
        if not os.path.exists(path):
            os.makedirs(path)
        return path

    def add_data(self, action, data_dict):
        # 為數據添加動作字段
        data_dict["action"] = action
        self.data.append(data_dict)

    def save_data_to_csv(self):
        if not self.data:
            print("No data to save.")
            return

        # 使用第一條數據的鍵作為欄位名稱
        keys = self.data[0].keys()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'collected_data_{timestamp}.csv'
        full_path = os.path.join(self.path, filename)

        with open(full_path, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=keys)
            writer.writeheader()
            for data_dict in self.data:
                writer.writerow(data_dict)

        print(f"Data stored in {full_path}")
