<template>
  <div>
    <div class="navbar-container">
      <font-awesome-icon v-if="left_icon" icon="chevron-left" @click="$router.push({ name: 'Home' })"/>
      <span @click="clickRightText" class="right-text">{{ right_text }}</span>
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
    right_text: String,
    type: String,
  },
  methods: {
    clickRightText() {
      if (this.type === 'control') {
        this.$router.push({name: 'IoTNew'})
      }
    }
  }
}
</script>

