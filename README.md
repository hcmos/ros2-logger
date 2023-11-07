# システム概要
| コンテナ |  |
| --- | --- |
| influxdb | ←(参照) grafana |
| ↑データ・コンフィグをバインドマウント | http://localhost:3000 |

  ↑  
influx dbクライアント ← トピック購読 ← 同ネットワーク出版

## 動作確認環境
- ubuntu 22.04(wsl2)
- docker for windows
- ROS 2 humble

# 使い方
### コンテナ立ち上げ
`docker-compose up (-d)`
### コンテナ停止
`docker-compose stop`
### コンテナ再起動
`docker-compose start`

## DBクライアントとログトピック購読の起動
`sh main.sh`
#### 停止 : SIGINT

# ROSトピック
一定の先頭文字列のトピックを出版することで保存する．周波数は出版者に合わされる．
### `/logf_`~ : [Float64MultiArray](https://docs.ros2.org/foxy/api/std_msgs/msg/Float64MultiArray.html)
numberというタグを持ち，要素を指定する．
### `/logs_`~ : [String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html)

# grafanaダッシュボード
- [作成方法説明動画(部内のみ)](https://kanazawa-it.box.com/s/9itengyy7x4wf3b1ny0t0b2cgt7bi4h4)
- テンプレート : `scripts/dashboard/topic_monitor.json`
