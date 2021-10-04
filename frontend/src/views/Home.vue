<template>
  <div>
    <Navbar :title="'Home'" :left_icon="false" :right_text="''"/>
    <div class="home-container">
      <HomeToday style="margin-bottom: 30px"/>
      <HomeSchedule style="margin-bottom: 30px"/>
      <HomeMenus style="margin-bottom: 30px"/>
      <div class="stt-container">
        <img src="@/assets/icons/mic.svg" alt="mic">
        <span class="text-darkgray font-400">터틀봇이 이야기를 감지중입니다.</span>
      </div>
    </div>

  </div>
</template>

<style lang="scss" scoped>
  @import "@/views/Home.scss";
</style>

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

