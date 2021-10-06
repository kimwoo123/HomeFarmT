<template>
  <div>

    <div class="today-weather">
      <img src="@/assets/icons/weather/sun.svg" alt="iot">
      <div class="text-box">
        <span class="today-date">2021년 2월 2일</span>
        <span class="weather-text">맑음</span>
      </div>
    </div>


    <div class="today-status-group">
      <div class="today-status">
        <span class="status-title">온도</span>
        <span class="status-value">22.5</span>
      </div>
      <div class="today-status">
        <span class="status-title">습도</span>
        <span class="status-value">22.5</span>
      </div>
      <div class="today-status">
        <span class="status-title">풍속</span>
        <span class="status-value">22.5</span>
      </div>
    </div>

  </div>
</template>

<script>
export default {
  name: 'HomeToday',

}
</script>

<style lang="scss" scoped>
  @import '@/components/Home/HomeToday.scss';
</style>