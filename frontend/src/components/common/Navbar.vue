<template>
  <div>
    <div class="navbar-container">
      <font-awesome-icon v-if="left_icon" icon="chevron-left" @click="$router.push({ name: 'Home' })" class="icon"/>
      <span class="right-text" @click="rightBtn()">{{ right_text }}</span>
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
  methods: {
    rightBtn() {
      if (this.right_text === '로그아웃') {
        this.logout()
      } else {
        this.$router.push({ name: this.right_push })
      }
    },
    logout() {
      sessionStorage.removeItem('token')
      sessionStorage.removeItem('email')
      this.$router.push({ name: 'Home' })
    },
  }
}
</script>

