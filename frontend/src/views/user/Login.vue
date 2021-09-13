<template>
  <div class="login-container">
    <h1>로그인</h1>
    
    <div class="email-form">
      <p style="width: 80px;">email</p>
      <input type="email" :value="email" class="login-input">
    </div>
    
    <div class="password-form">
      <p style="width: 80px;">password</p>
      <input type="password" :value="password" @keyup.enter="requestLogin" class="login-input">
    </div>
    
    <button class="login-button" @click="requestLogin">Login</button>
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
      const loginInfo = {
        email: this.email,
        passwrod: this.password,
      }
      this.store.dispatch('user/requestLogin', loginInfo)
    }
  }
}
</script>


