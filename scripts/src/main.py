# Copyright 2022 iwatake2222
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
import argparse
import subprocess
from datetime import datetime
import re
import threading
from multiprocessing import Process
import influxdb_client

import hz
import data


class InfluxDbAccessor:
    def __init__(self, url: str, token: str, org: str, bucket_name: str):
        self.url = url
        self.token = token
        self.org = org
        self.bucket_name = bucket_name

    def create_bucket(self):
        with influxdb_client.InfluxDBClient(url=self.url, token=self.token) as client:
            buckets_api = client.buckets_api()
            buckets = buckets_api.find_buckets().buckets
            my_buckets = [bucket for bucket in buckets if bucket.name==self.bucket_name]
            _ = [buckets_api.delete_bucket(my_bucket) for my_bucket in my_buckets]
            _ = buckets_api.create_bucket(bucket_name=self.bucket_name, org=self.org)

    def write_point(self, measurement_datetime, field: str, topic_name: str, number: int, value: float):
        with influxdb_client.InfluxDBClient(url=self.url, token=self.token, org=self.org) as client:
            point_settings = influxdb_client.client.write_api.PointSettings()
            write_api = client.write_api(write_options=influxdb_client.client.write_api.SYNCHRONOUS, point_settings=point_settings)

            if(number is None):
                p = influxdb_client.Point('ros2_topic')\
                    .tag('topic_name', topic_name)\
                    .time(measurement_datetime)
            else:
                p = influxdb_client.Point('ros2_topic')\
                    .tag('topic_name', topic_name)\
                    .tag('number', number)\
                    .time(measurement_datetime)

            write_api.write(bucket=self.bucket_name, record=p.field(field, value))


db: InfluxDbAccessor = None


def update_hz_cb(hz_dict: dict[str, float]):
    def run(hz_dict):
        measurement_datetime = int(datetime.now().timestamp() * 1e9)
        for topic, hz in hz_dict.items():
            # print(f'{topic}: {hz: .03f} [Hz]')
            db.write_point(measurement_datetime, 'topic_rate_hz', topic, None, hz)
    thread = threading.Thread(target=run, args=(hz_dict, ))
    thread.start()

def update_float_cb(float_dict: dict[(str, int), float]):
    def run(float_dict):
        measurement_datetime = int(datetime.now().timestamp() * 1e9)
        for key, data in float_dict.items():
            db.write_point(measurement_datetime, 'topic_data_float', key[0], key[1], data)
    thread = threading.Thread(target=run, args=(float_dict, ))
    thread.start()

def update_str_cb(str_dict: dict[str, str]):
    def run(str_dict):
        measurement_datetime = int(datetime.now().timestamp() * 1e9)
        for topic, data in str_dict.items():
            db.write_point(measurement_datetime, 'topic_data_str', topic, None, data)
    thread = threading.Thread(target=run, args=(str_dict, ))
    thread.start()


def subscribe_hz(topic_list: list[str], window_size: int):
    hz_verb = hz.HzVerb()
    parser = argparse.ArgumentParser()
    hz_verb.add_arguments(parser, 'ros2_monitor_grafana')
    args = parser.parse_args('')
    args.topic_list = topic_list
    args.window_size = window_size

    args.update_hz_cb = update_hz_cb
    hz.main(args)
    # not reached here

def subscribe_data(topic_list: list[str], window_size: int):
    parser = argparse.ArgumentParser()
    args = parser.parse_args('')
    args.topic_list = topic_list
    args.window_size = window_size

    args.update_float_cb = update_float_cb
    args.update_str_cb = update_str_cb
    data.main(args)


def make_topic_list(ignore_regexp: str, target_regexp: str) -> list[str]:
    topic_list = []
    topic_list = subprocess.run(['ros2', 'topic', 'list'],
                                capture_output=True,
                                text=True)
    topic_list = topic_list.stdout.splitlines()
    topic_list = [topic for topic in topic_list if \
        re.search(target_regexp, topic) and not re.search(ignore_regexp, topic)]

    len_topic_list = len(topic_list)
    print(topic_list)
    print(f'上記の周波数を記録するトピック数: {len_topic_list}')
    if len_topic_list > 50:
        print('Warning: too many topics are monitored. Result may not be accurate. '+
        ' Please consider to use ignore_regexp and target_regexp option')

    return topic_list

def make_log_topic_list(topic_list: list[str]):
    log_topic_list = [s for s in topic_list if s.startswith('/log')]
    print(log_topic_list)
    print(f'上記のデータを記録するトピック数: {len(log_topic_list)}')
    return log_topic_list



def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ignore_regexp', type=str, default='(parameter_events|rosout|debug|tf)')
    parser.add_argument('--target_regexp', type=str, default='.*')
    parser.add_argument('--window_size', type=int, default=10)
    parser.add_argument('--token', type=str, default='my-super-secret-auth-token',
                        help='InfluxDB Token')
    parser.add_argument('--org', type=str, default='my-org',
                        help='InfluxDB Organization')
    parser.add_argument('--url', type=str, default='http://localhost:8086',
                        help='InfluxDB URL')
    parser.add_argument('--bucket_name', type=str, default='my-bucket',
                        help='InfluxDB Bucket Name')
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    global db
    db = InfluxDbAccessor(args.url, args.token, args.org, args.bucket_name)
    db.create_bucket()

    print('\033[32m'+'注意'+'\033[0m' + ': 下記にWARNINGが発生したものは周波数を取ることが出来ません.問題がある場合は再実行してください.')

    topic_list = make_topic_list(args.ignore_regexp, args.target_regexp)
    log_topic_list =make_log_topic_list(topic_list)

    p_hz = Process(target=subscribe_hz, args=(topic_list, args.window_size))
    p_data = Process(target=subscribe_data, args=(log_topic_list, args.window_size))
    p_hz.start()
    p_data.start()


if __name__ == '__main__':
    main()
