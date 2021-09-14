<template>
  <div class="home-conatiner">
    <HomeBotStatus/>
  </div>
</template>
<style lang="scss" scoped>
  @import url('./Home.scss');
</style>
<script>
import HomeBotStatus from '../components/Home/HomeBotStatus.vue'
export default {
  name: "Home",
  components: {
    HomeBotStatus,
  }

}
</script>

