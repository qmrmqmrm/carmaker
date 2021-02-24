#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import os.path as op
import pandas as pd
from config import Config


def get_csv():
	# Global path info
	cfg = Config()
	df_TM = pd.read_csv(cfg.USEPATH)

	df_tf = {}
	df_dic = {}
	# key_name = ['0', '1']
	key_name = ['x', 'y']
	for key in key_name:
		df_dic[key] = []
		df_tf[key] = []

	# df_tf['x']=df_TM['0'].to_dict()
	df_tf['x'] = df_TM['x'].to_dict()
	# df_tf['y']=df_TM['1'].to_dict()
	df_tf['y'] = df_TM['y'].to_dict()

	df_dic['x'] = df_tf['x'].values()
	df_dic['y'] = df_tf['y'].values()

	return df_dic
