use esp_hal::rtc_cntl::Rtc;
pub struct Random<'a> {
    rtc: &'a Rtc<'static>,
}

impl<'a> Random<'a> {
    pub fn new(rtc: &'a Rtc<'static>) -> Self {
        Random {
            rtc,
        }
    }
    // 伪随机
    pub fn get_rand(&self,num:u64)->u64{
        if self.rtc.get_time_us()%2==0{
            (self.rtc.get_time_us()+self.rtc.get_time_ms()-num) %num 
        }else if self.rtc.get_time_us()%3==0{
            (self.rtc.get_time_us()+ num +self.rtc.get_time_ms()*num) %num 
        }else{
            (self.rtc.get_time_raw()*num+self.rtc.get_time_ms()+num) %num 
        }
    }

}