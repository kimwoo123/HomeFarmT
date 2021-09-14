<template>
  <div class="home-conatiner">
    <Navbar/>
    <HomeBotStatus/>
  </div>
</template>
<style>
  @import url('./Home.scss');
</style>
<script>
import Navbar from '../components/Home/Navbar.vue'
import HomeBotStatus from '../components/Home/HomeBotStatus.vue'
export default {
  name: "Home",
  components: {
    Navbar,
    HomeBotStatus,
  }

}
</script>

