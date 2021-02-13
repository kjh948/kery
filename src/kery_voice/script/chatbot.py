# -*- coding: utf-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
from chatterbot import ChatBot
import logging
from chatterbot.response_selection import get_random_response

# Enable info level logging
logging.basicConfig(level=logging.INFO)

class r2d2Bot(object):

    def __init__(self, dbpath=None, corpus = './chatterbot/corpus/',read_only=True):
        #open db file here
        #try:
            if dbpath is not None:
                self.bot_engine = ChatBot(
                    'R2D2',
                    storage_adapter="chatterbot.storage.SQLStorageAdapter",
                    response_selection_method=get_random_response,
                    database=dbpath,
                    read_only = read_only
                )
                print('chatbot DB loaded successfully')
            else:
                self.bot_engine = ChatBot(
                    'R2D2',
                    storage_adapter="chatterbot.storage.SQLStorageAdapter",
                    trainer='chatterbot.trainers.ChatterBotCorpusTrainer',
                    response_selection_method=get_random_response,
                    database=None,
                    read_only=read_only
                )
                self.bot_engine.train(
                    corpus
                )
                self.bot_engine.trainer.export_for_training(file_path='out.json')
                print('chatbot corpus loaded successfully')
        #except:
        #    print('Error with DB loading')

    def get_response(self, query):
        response = self.bot_engine.get_response(query)
        print("Bot: " + response.text)
        return response


if __name__ == "__main__":
    bot = r2d2Bot(corpus='./resources', read_only=False)
    while True:
        query = raw_input("Tell me : ")
        response = bot.get_response(query)
        #print("Bot: " + response.text)
        # print(response)
