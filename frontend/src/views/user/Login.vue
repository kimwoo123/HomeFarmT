<template>
  <div class="login-container">
    <div class="nav-container">
      <span>Login</span>
    </div>

    <div class="logo"></div>

    <div class="input-container">
      <font-awesome-icon icon="user" class="icon"/>
      <input type="email" v-model="email" placeholder="email">
      <font-awesome-icon icon="lock" class="icon"/>
      <input type="password" v-model="password" @keyup.enter="requestLogin" placeholder="password">
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
    requestLogin() {
      this.$apollo.mutate({
        mutation: gql`mutation ($email: String!, $password: String!) {
          login(email: $email , password: $password) {
            email
            token
          }
        }`,
        variables: {
          email: this.email,
          password: this.password
        }
        })
        .then((res) => {
          console.log(res.data)
          localStorage.setItem('token', res.data.login.token)
        })
        .catch((err) => {
          console.log(err, 'no')
        })
      },
    },
  }
</script>


