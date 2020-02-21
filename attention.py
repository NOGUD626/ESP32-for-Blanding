#!usr/bin/env python
# -*- coding: utf-8 -*- 

import paho.mqtt.client as mqtt     # MQTTのライブラリをインポート
import ast
import datetime
import time
import requests

# JSONフォーマット用辞書変数
json_tmp = {}
i =0
dicH = []
dicR = []
dicP = []
t1 = 0
flag = False
# ブローカーに接続できたときの処理
def on_connect(client, userdata, flag, rc):
    print("Connected with result code " + str(rc))  # 接続できた旨表示
    client.subscribe("topic")  # subするトピックを設定 

# ブローカーが切断したときの処理
def on_disconnect(client, userdata, flag, rc):
    if  rc != 0:
        print("Unexpected disconnection.")

# メッセージが届いたときの処理
def on_message(client, userdata, msg):
    # msg.topicにトピック名が，msg.payloadに届いたデータ本体が入っている
    Format_to_JSON(msg.payload,msg.topic)

# 受信したMQTTのペーロードを一つの辞書ファイルに作り変えるところ
def Format_to_JSON(payload,topic):
    global json_tmp
    flag = False
    text = str(payload.decode(encoding='utf-8'))
    if(text == 'START'):
        flag = True
        json_tmp = {}
    elif(text != 'END' and text != 'START' and text != 'payload'):
        flag = True
        dic = ast.literal_eval(text)
        json_tmp.update(dic)
    elif(text == 'END'):
        flag = False
        dt_now = datetime.datetime.now()
        del json_tmp['type']
        database_insert_json(json_tmp,dt_now)

# 一つにまとめたJSONファイルをモンゴに登録するところ
def database_insert_json(dic,date):
    global i
    global dicH
    global dicR
    global dicP
    global t1
    global flag
    t2 = time.time()
    if(i <= 10):
        dicH.append(int(dic["ax"]))
        dicR.append(int(dic["ay"]))
        dicP.append(int(dic["az"]))
        i += 1
    elif(i == 11):
        dicH.append(int(dic["ax"]))
        dicR.append(int(dic["ay"]))
        dicP.append(int(dic["az"]))
        dicH = dicH[-10:]
        dicR = dicR[-10:]
        dicP = dicP[-10:]
        total = abs(sum(dicH)/10 - dic["ax"])+abs(sum(dicR)/10  - dic["ay"])+abs(sum(dicP)/10 - dic["az"])
        print(total)
        if(total >= 2):
            t1 = time.time()
            flag = True
            headers = {
                'content-type': 'application/json',}
            data = '{  "bp_id": "16ece22bac217",  "point_id": "16e3a5e6996233",  "ch": "s20"}'
            response = requests.put(
                'https://me.medisec.io/point',
                headers=headers, data=data )
            print('異常')
        else:
            if(flag and int(t2-t1) > 30):
                flag = False
                headers = {'content-type': 'application/json',}
                data = '{  "bp_id": "16ece22bac217",  "point_id": "16e3a5e6996233",  "ch": "s11"}'
                response = requests.put(
                    'https://me.medisec.io/point',
                    headers=headers, data=data )
            else:
                print("")
    else:
        i = 11 
            
    
if __name__ == '__main__':
    client = mqtt.Client()                 # クラスのインスタンス(実体)の作成
    client.on_connect = on_connect         # 接続時のコールバック関数を登録
    client.on_disconnect = on_disconnect   # 切断時のコールバックを登録
    client.on_message = on_message         # メッセージ到着時のコールバック

    client.connect("192.168.13.2", 1883, 60)  # 接続先は自分自身

    client.loop_forever()                  # 永久ループして待ち続ける
