<template>
  <div>

    <div class="today-weather">
      <img src="@/assets/icons/weather/sun.svg" alt="iot">
      <div class="text-box">
        <span class="today-date">{{ todayDate }}</span>
        <span class="weather-text">{{ weather }}</span>
      </div>
      <div class="text-box">
        <span class="d">{{ region }}</span>
        <img :src="this.iconURL" alt="">
      </div>
    </div>


    <div class="today-status-group">
      <div class="today-status">
        <span class="status-title">온도</span>
        <span class="status-value">{{ temp }}°C</span>
      </div>
      <div class="today-status">
        <span class="status-title">습도</span>
        <span class="status-value">{{ humidity }}%</span>
      </div>
      <div class="today-status">
        <span class="status-title">풍속</span>
        <span class="status-value">{{ windSpeed }}m/s</span>
      </div>
    </div>

  </div>
</template>

<script>
import axios from 'axios'
import weatherTranslate from './WeatherTranslate'

export default {
  name: 'HomeToday',
  data() {
    return {
      todayDate: '',
      weather: '',
      temp: '',
      humidity: '',
      windSpeed: '',
      region: '',
      iconURL: '',
      regionTranslate: {
        '대전' : 'Daejeon',
        '서울' : 'Seoul', 
        '구미' : 'Gumi',
        '광주' : 'Gwangju', 
        '부산' : 'Busan',
      }
    }
  },
  created() {
    const cityname = this.regionTranslate[sessionStorage.getItem('region')] ||'Daejeon'
    const APIkey = process.env.VUE_APP_WEATHER_API
    const baseURL = `https://api.openweathermap.org/data/2.5/weather?q=${cityname}&appid=${APIkey}`
    axios.get(baseURL)
    .then(res => {
      const today = new Date(res.data.dt * 1000)
      this.todayDate = today.getFullYear() + '년' + (today.getMonth() + 1) + '월' + today.getDate() + '일'
      this.temp = parseInt(res.data.main.temp - 273.15)
      this.humidity = res.data.main.humidity
      this.windSpeed = res.data.wind.speed
      this.region = sessionStorage.getItem('region') || '대전'
      const weatherDesc = res.data.weather[0].id
      this.weather = weatherTranslate[weatherDesc]
      this.iconURL = `http://openweathermap.org/img/wn/${res.data.weather[0].icon}@2x.png`
    })
    .catch(err => {
      console.log(err)
    })
  },
}
</script>

<style lang="scss" scoped>
  @import '@/components/Home/HomeToday.scss';
</style>