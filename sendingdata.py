def connectToWifi(sta_if):

    sta_if.active(True)
    sta_if.connect('EEERover','exhibition')
    while(not sta_if.isconnected()):
        pass
    time.sleep(0.5)

    
