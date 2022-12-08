
unique = True



with open("lookup_table_out.csv", "ab") as fout:
    # first file:
    # with open("table_221121_142312__no_reset_0", "rb") as f:
    #     fout.writelines(f)
    # # now the rest:    
    for num in range(0, 13):
        filename = "table_221121_142312__no_reset_"+str(num)
        print(filename)
        with open(filename, "rb") as f:
            # next(f) # skip the header, portably
            fout.writelines(f)
            
            
            
