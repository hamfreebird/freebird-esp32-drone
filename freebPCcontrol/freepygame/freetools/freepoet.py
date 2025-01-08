"""free post tool"""

import pathlib
import random

# 各词库的路径
ChineseSemanticKB_path = "D:\\Dev\\ChineseSemanticKB"
ChineseSemanticKB_dict_path = "D:\\Dev\\ChineseSemanticKB\\dict"

# 各词库的信息加载
ChineseSemanticKB_animal_list = pathlib.Path(ChineseSemanticKB_path + "\\THUOCL_animal.txt")  # 动物
animal_list = ChineseSemanticKB_animal_list.read_text(encoding='utf-8').split("\n")
animal_list_len = len(animal_list)
ChineseSemanticKB_dict_degree_adverb_list = pathlib.Path(ChineseSemanticKB_dict_path + "\\degree_adverb.txt")  # 情态副词
degree_adverb_list = ChineseSemanticKB_dict_degree_adverb_list.read_text(encoding="utf-8").split("\n")
degree_adverb_list_len = len(degree_adverb_list)

# 组句
final_sentence = ""
animal_index = random.randrange(0, animal_list_len)
final_sentence += animal_list[animal_index]
degree_adverb_index = random.randrange(0, degree_adverb_list_len)
final_sentence += degree_adverb_list[degree_adverb_index]
animal_index = random.randrange(0, animal_list_len)
final_sentence += animal_list[animal_index]
print(final_sentence)
