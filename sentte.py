import requests

headers = {
    'content-type': 'application/json',
}

data = '{  "bp_id": "16ece22bac217",  "point_id": "16e3a5e6996233",  "ch": "s20"}'

def main():
    #GETパラメータはparams引数に辞書で指定する
    response = requests.put(
        'https://me.medisec.io/point',
        headers=headers, data=data )
    
if __name__== '__main__':
    main()
