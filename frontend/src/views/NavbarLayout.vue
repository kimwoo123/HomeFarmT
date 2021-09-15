<template>
  <div>
    <div class="nav-container">
      <font-awesome-icon icon="arrow-left" @click="$router.push({ name: 'Home' })" class="icon"/>
      <div>
        <span>Turtlebot</span><span class="router-name">{{ currentRouteName }}</span>
      </div>
      <font-awesome-icon icon="user" class="icon"/>
    </div>
    <router-view></router-view>
  </div>
</template>

<style lang="scss" scoped>
  @import '@/views/NavbarLayout.scss';
</style>

<script>
export default {
  name: 'NavbarLayout',
  computed: {
    currentRouteName() {
      return this.$route.name;
    }
  },
  mounted() {
    const currentRouteName = document.querySelector('.router-name')
    switch (this.currentRouteName) {
      case 'IoT':
        console.log('IoT')
        currentRouteName.style.color = "#EBBBA6"
        break;
      case 'Patrol':
        console.log('Patrol')
        currentRouteName.style.color = "#95BFC2"
        break;
      case 'Schedule':
        console.log('Schedule')
        currentRouteName.style.color = "#EBBBA6"
        break;
    }
  }
}
</script>