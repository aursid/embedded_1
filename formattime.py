#formattime functions
def display_date_time_tuple(timetuple):

    date_string = ("%d:%d:%d %d/%d/%d" %(timetuple[4],timetuple[5],timetuple[6],timetuple[2],timetuple[1],timetuple[0]))
    return date_string

def convert_time(time_list):
    for k in range(0, len(time_list)):
        time_list[k] = display_date_time_tuple(time_list[k])
    return time_list
