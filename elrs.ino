void elrsinit(){
  crsf = new CRSFforArduino(&Serial6, PD6,PD5); // define Serial1 pin in this section too
    if (!crsf->begin())
    {
        crsf->end();

        delete crsf;
        crsf = nullptr;

        // Serial1.println("CRSF for Arduino initialisation failed!");
        while (1)
        {
            // delay(10);
        }
    }
    rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;
}

void elrsupdate(){
  crsf->update();
  roll_channel = crsf->rcToUs(crsf->getChannel(1));
  pitch_channel = crsf->rcToUs(crsf->getChannel(2));
  throttle_channel = crsf->rcToUs(crsf->getChannel(3));
  yaw_channel = crsf->rcToUs(crsf->getChannel(4));
  ch5_channel = crsf->rcToUs(crsf->getChannel(5));
  ch6_channel = crsf->rcToUs(crsf->getChannel(6));
  ch7_channel = crsf->rcToUs(crsf->getChannel(7));
  ch8_channel = crsf->rcToUs(crsf->getChannel(8));
}
