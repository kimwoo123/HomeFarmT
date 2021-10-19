<template>
  <div>
    <div class="navbar-container">
      <div class="box"></div>
      <img :src="require('@/assets/images/logo.png')" alt="logo" class="logo">
      <font-awesome-icon  icon="user" @click="$router.push({ name: 'My' })" class="icon"/>
    </div>

    <div class="home-container">
      <HomeToday style="margin-bottom: 30px"/>
      <HomeSchedule style="margin-bottom: 35px"/>
      <HomeMenus style="margin-bottom: 25px"/>
      <div class="stt-container">
        <img src="@/assets/icons/mic.svg" alt="mic">
        <span class="font-400 listening">터틀봇이 이야기를 감지중입니다.</span>
      </div>
    </div>

  </div>
</template>

<script>
import HomeMenus from '@/components/Home/HomeMenus.vue'
import HomeSchedule from '@/components/Home/HomeSchedule.vue'
import HomeToday from '@/components/Home/HomeToday.vue'

export default {
  name: "Home",
  components: {
    HomeMenus,
    HomeSchedule,
    HomeToday
  },
}
</script>

<style lang="scss" scoped>
  @import "./Home.scss";
</style>