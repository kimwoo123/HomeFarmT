<template>
  <div class="login-wrapper">
    <h1>로그인</h1>
    <input type="email" :value="email" class="login-email-input">
    <input type="password" :value="password" class="login-password-input">
    <button class="login-button">Login</button>
  </div>
</template>
<style lang="scss" scoped>
  @import url('./Login.scss');
</style>
<script>
export default {
  name: 'Login',
  data() {
    return {
      email: '',
      password: '',
    }
  },
  methods: {
    requestLogin() {
      
    }
  }
}
</script>


