<template>
  <div class="home-conatiner">
    <Navbar/>
    <HomeBotStatus/>
    <div class="home-mid-side">
      <HomeBotCam/>
      <HomeMenus/>
    </div>
    <HomeBotCurrentTask/>
  </div>
</template>
<style lang="scss" scoped>
  @import url('./Home.scss');
</style>
<script>
import Navbar from '../components/Home/Navbar.vue'
import HomeBotStatus from '../components/Home/HomeBotStatus.vue'
import HomeBotCam from '../components/Home/HomeBotCam.vue'
import HomeMenus from '../components/Home/HomeMenus.vue'
import HomeBotCurrentTask from '../components/Home/HomeBotCurrentTask.vue'

export default {
  name: "Home",
  components: {
    Navbar,
    HomeBotStatus,
    HomeBotCam,
    HomeMenus,
    HomeBotCurrentTask,
  }

}
</script>

