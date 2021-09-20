module.exports = (sequelize, DataTypes) =>
sequelize.define('users', {
  email: { type: DataTypes.STRING, primaryKey: true },
  hashid: { type: DataTypes.STRING },
  password: { type: DataTypes.STRING, defaultValue: 'none' }
}, {
  timestamps: false
});
