<template>
  <div class="login-container">
    <div class="nav-container">
      <span>Login</span>
    </div>

    <div class="logo"></div>

    <div class="input-container">
      <font-awesome-icon icon="user" class="icon"/>
      <input type="email" :value="email" placeholder="email">
      <font-awesome-icon icon="lock" class="icon"/>
      <input type="password" :value="password" @keyup.enter="requestLogin" placeholder="password">
      <div class="option">
        <small>Forgot Password?</small>
        <small @click="$router.push({ name: 'Signup' })">Sign Up</small>
      </div>
    </div>

    <button @click="requestLogin">Login</button>
  </div>
</template>

<style lang="scss" scoped>
  @import './Login.scss';
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


