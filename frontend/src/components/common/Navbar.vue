<template>
  <div>
    <div class="navbar-container">
      <font-awesome-icon v-if="left_icon" icon="chevron-left" @click="$router.push({ name: 'Home' })" class="icon"/>
      <span class="right-text" @click="$router.push({ name: right_push })">{{ right_text }}</span>
    </div>
    <p class="title">{{ title }}</p>
  </div>
</template>

<style lang="scss" scoped>
  @import './Navbar.scss';
</style>

<script>
export default {  
  name: 'Navbar',
  props: {
    title: String,
    left_icon: Boolean,
    left_push: String,
    right_text: String,
    right_push: String,
  },
  data() {
    return {}
  }
}
</script>

