import urllib
import json
import os
from flask import Flask, request, make_response, jsonify

import pprint 
# initialize the flask app
app = Flask(__name__)

# default route
@app.route('/')
def index():
    print("got request")
    return 'Hello World!'

# create a route for webhook
@app.route('/webhook', methods=['GET', 'POST'])
def webhook():
    
    req = request.get_json(force=True)
    pprint.pprint(req)
    intent = req['queryResult']['intent'] #1

    print('You said:'+req['queryResult']['queryText'])
    # if action == 'interest':
    #     name = req['queryResult']['parameters']['roominfomation']  #2
        
    # else:
    #     return "test"
    #print(action)

    return {'fulfillmentText': intent} #3

# run the app
if __name__ == '__main__':
   port = int(os.getenv('PORT',5000))
   #app.run(debug=True, ssl_context=('keys/cert.pem', 'keys/key.pem'), port=port,host='0.0.0.0')
   #app.run(debug=True, ssl_context=('keys/server.crt', 'keys/server.key'), port=port,host='0.0.0.0')
   app.run(debug=True, port=port,host='0.0.0.0')
