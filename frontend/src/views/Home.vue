<template>
  <div>
    <Navbar :title="'메인'" :left_icon="false" :right_text="''"/>
    <div class="home-container">
      <HomeToday style="margin-bottom: 30px"/>
      <HomeSchedule style="margin-bottom: 30px"/>
      <HomeMenus style="margin-bottom: 30px"/>
      <div class="stt-container">
        <img src="@/assets/icons/mic.svg" alt="mic">
        <span class="font-400 listening">터틀봇이 이야기를 감지중입니다.</span>
      </div>
    </div>

  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import HomeMenus from '@/components/Home/HomeMenus.vue'
import HomeSchedule from '@/components/Home/HomeSchedule.vue'
import HomeToday from '@/components/Home/HomeToday.vue'

export default {
  name: "Home",
  components: {
    Navbar,
    HomeMenus,
    HomeSchedule,
    HomeToday
  }
}
</script>

<style lang="scss" scoped>
  @import "./Home.scss";
</style>