<template>
  <div class="home-conatiner">
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
import HomeBotStatus from '../components/Home/HomeBotStatus.vue'
import HomeBotCam from '../components/Home/HomeBotCam.vue'
import HomeMenus from '../components/Home/HomeMenus.vue'
import HomeBotCurrentTask from '../components/Home/HomeBotCurrentTask.vue'

export default {
  name: "Home",
  components: {
    HomeBotStatus,
    HomeBotCam,
    HomeMenus,
    HomeBotCurrentTask,
  }

}
</script>

