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
import gql from 'graphql-tag'

export default {
  name: 'Login',
  data() {
    return {
      email: '',
      password: '',
    }
  },
  methods: {
    async requestLogin() {
      await this.$apollo.mutate({
        mutation: gql`mutation ($email: String!, $password: String) {
          login(email: $email , password: $password) {
            email
            password
          }
        }`,
        variables: {
          email: this.email,
          password: this.password
        }
        })
      // this.store.dispatch('user/requestLogin', loginInfo)
    }
  }
}
</script>


