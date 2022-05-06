import yaml

file_name = input("enter name of file, ctrl c to quit \n")

def read_from_file():
    with open(file_name) as file:
        documents = yaml.full_load(file)

        for item, doc in documents.items():
            print(item, ":", doc)

def write_to_file():
    dict_file = [{'sports' : ['soccer', 'football', 'basketball', 'cricket', 'hockey', 'table tennis']},
                {'countries' : ['Pakistan', 'USA', 'India', 'China', 'Germany', 'France', 'Spain']}]

    with open(file_name, 'w') as file:
        documents = yaml.dump(dict_file, file)

write_to_file()
